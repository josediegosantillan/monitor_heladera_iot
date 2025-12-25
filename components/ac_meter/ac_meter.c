#include "ac_meter.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "AC_METER";
static adc_oneshot_unit_handle_t s_adc_handle = NULL; // Referencia prestada
static ac_meter_cfg_t s_cfg;
static int *s_buffer = NULL;
static int s_buffer_len = 0;

esp_err_t ac_meter_init(adc_oneshot_unit_handle_t adc_handle, const ac_meter_cfg_t *cfg) {
    if (adc_handle == NULL || cfg == NULL) return ESP_ERR_INVALID_ARG;

    s_adc_handle = adc_handle;
    s_cfg = *cfg;

    if (cfg->fs_hz <= 0 || cfg->fs_hz > 1000000 || cfg->window_ms <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = cfg->bitwidth,
        .atten = cfg->atten,
    };
    
    // Solo configuramos el CANAL en la unidad existente
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, cfg->channel, &config));
    
    ESP_LOGI(TAG, "Canal %d configurado (ADC compartido)", cfg->channel);
    return ESP_OK;
}

esp_err_t ac_meter_read(ac_meter_reading_t *out) {
    if (s_adc_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    int samples = (s_cfg.fs_hz * s_cfg.window_ms) / 1000;
    int delay_us = 1000000 / s_cfg.fs_hz;
    long sum = 0;

    if (samples <= 0 || delay_us <= 0) return ESP_ERR_INVALID_ARG;

    if (samples > s_buffer_len) {
        int *new_buf = realloc(s_buffer, samples * sizeof(int));
        if (!new_buf) return ESP_ERR_NO_MEM;
        s_buffer = new_buf;
        s_buffer_len = samples;
    }
    int *buffer = s_buffer;

    // 1. Offset DC
    for (int i = 0; i < samples; i++) {
        int raw;
        esp_err_t err = adc_oneshot_read(s_adc_handle, s_cfg.channel, &raw);
        if (err != ESP_OK) return err;
        buffer[i] = raw;
        sum += raw;
        esp_rom_delay_us(delay_us);
    }
    float offset = (float)sum / samples;

    // 2. RMS
    float sum_sq = 0;
    for (int i = 0; i < samples; i++) {
        float val = buffer[i] - offset;
        sum_sq += val * val;
    }
    
    float rms_raw = sqrt(sum_sq / samples);
    
    // CALIBRACION VOLTAJE
    float volts_adc = (rms_raw * 3.3f) / 4095.0f;
    out->vline_rms = volts_adc * 1065.0f; // Ajustar factor con multímetro
    out->raw_rms = rms_raw;

    return ESP_OK;
}
