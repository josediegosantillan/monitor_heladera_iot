#include "zmct103c.h"
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ZMCT";

// Helper de Calibración (Misma lógica que en ac_meter)
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit, .chan = chan, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif
    *out_handle = handle;
    return calibrated;
}

esp_err_t zmct103c_init(zmct103c_t *ctx, zmct103c_cfg_t *cfg) {
    ESP_RETURN_ON_FALSE(ctx && cfg && cfg->adc_handle, ESP_ERR_INVALID_ARG, TAG, "Args Invalidos");
    
    // --- NUEVA VALIDACIÓN (V3.0) ---
    if (cfg->burden_ohms <= 0.001f) {
        ESP_LOGE(TAG, "Burden Ohms invalido (evitando div/0)");
        return ESP_ERR_INVALID_ARG;
    }
    // -------------------------------

    ctx->cfg = *cfg;
    
    // Configurar Canal sobre handle existente
    adc_oneshot_chan_cfg_t c_cfg = {
        .atten = ADC_ATTEN_DB_12, 
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(cfg->adc_handle, cfg->channel, &c_cfg), TAG, "Chan Init Fail");

    ctx->cali_enable = adc_calibration_init(cfg->unit_id, cfg->channel, ADC_ATTEN_DB_12, &ctx->cali_handle);
    return ESP_OK;
}

esp_err_t zmct103c_read_rms(zmct103c_t *ctx, float *irms_out) {
    ESP_RETURN_ON_FALSE(ctx && irms_out, ESP_ERR_INVALID_ARG, TAG, "Args NULL");
    
    int samples = 400; 
    double sum_sq = 0, sum = 0;
    int raw, mv;
    int64_t t_period = 500; 
    int64_t next = esp_timer_get_time();

    // 1. Offset
    for(int i=0; i<samples; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_read(ctx->cfg.adc_handle, ctx->cfg.channel, &raw), TAG, "Read Fail");
        if (ctx->cali_enable) adc_cali_raw_to_voltage(ctx->cali_handle, raw, &mv);
        else mv = raw; 
        
        sum += mv;
        next += t_period;
        while(esp_timer_get_time() < next);
    }
    double offset = sum / samples;

    // 2. RMS
    next = esp_timer_get_time();
    for(int i=0; i<samples; i++) {
        adc_oneshot_read(ctx->cfg.adc_handle, ctx->cfg.channel, &raw);
        if (ctx->cali_enable) adc_cali_raw_to_voltage(ctx->cali_handle, raw, &mv);
        else mv = raw;

        double val = (double)mv - offset;
        sum_sq += val * val;

        next += t_period;
        while(esp_timer_get_time() < next);
    }
    
    float v_rms_mv = sqrt(sum_sq / samples);
    *irms_out = (v_rms_mv / 1000.0f) / ctx->cfg.burden_ohms * ctx->cfg.ct_ratio;
    
    return ESP_OK;
}
