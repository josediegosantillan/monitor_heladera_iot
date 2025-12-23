#include "ac_meter.h"
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "AC_METER";

static adc_oneshot_unit_handle_t s_adc = NULL; // Puntero al handle compartido
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_enable = false;
static ac_meter_cfg_t s_cfg;
static float s_k_line = 1.0f; 

// --- Calibración Portable ---
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Usando Scheme: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = chan,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Usando Scheme: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    if (!calibrated) ESP_LOGW(TAG, "Calibracion no soportada por Hardware");
    
    return calibrated;
}

esp_err_t ac_meter_init(const ac_meter_cfg_t *cfg) {
    ESP_RETURN_ON_FALSE(cfg && cfg->adc_handle, ESP_ERR_INVALID_ARG, TAG, "Config/Handle NULL");
    s_cfg = *cfg;
    s_adc = cfg->adc_handle; // Guardamos referencia al handle del Main

    // Configurar SOLO el canal (La unidad ya existe)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = s_cfg.atten,
        .bitwidth = s_cfg.bitwidth,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, s_cfg.channel, &chan_cfg), TAG, "Config Channel Fail");

    // Iniciar Calibración
    s_cali_enable = adc_calibration_init(s_cfg.unit_id, s_cfg.channel, s_cfg.atten, &s_cali_handle);

    return ESP_OK;
}

void ac_meter_set_k(float k) { if(k > 0) s_k_line = k; }

esp_err_t ac_meter_read(ac_meter_reading_t *out) {
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "Buffer NULL");
    
    int samples = (s_cfg.fs_hz * s_cfg.window_ms) / 1000;
    int64_t period_us = 1000000 / s_cfg.fs_hz;
    
    double sum = 0, sum_sq = 0;
    int raw, mv;
    int64_t next_time = esp_timer_get_time();

    // 1. Offset
    for (int i = 0; i < samples; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, s_cfg.channel, &raw), TAG, "Read ADC Fail");
        if (s_cali_enable) {
            ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali_handle, raw, &mv), TAG, "Cali Fail");
        } else { mv = raw; }
        
        sum += mv;
        next_time += period_us;
        while(esp_timer_get_time() < next_time);
    }
    double offset = sum / samples;
    out->offset_mv = (float)offset;

    // 2. RMS
    next_time = esp_timer_get_time();
    for (int i = 0; i < samples; i++) {
        adc_oneshot_read(s_adc, s_cfg.channel, &raw);
        if (s_cali_enable) adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        else mv = raw;

        double val = (double)mv - offset;
        sum_sq += val * val;

        next_time += period_us;
        while(esp_timer_get_time() < next_time);
    }
    
    float v_rms_mv = sqrt(sum_sq / samples);
    out->vline_rms = (v_rms_mv / 1000.0f) * s_k_line; 
    
    return ESP_OK;
}
