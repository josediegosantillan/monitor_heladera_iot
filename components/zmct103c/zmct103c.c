#include "zmct103c.h"
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_rom_sys.h"

#define AC_LINE_FREQ_HZ 50

static int *s_buffer = NULL;
static int s_buffer_len = 0;

esp_err_t zmct103c_init(zmct103c_t *ctx, adc_oneshot_unit_handle_t handle, const zmct103c_cfg_t *cfg) {
    if (!ctx || !handle || !cfg) return ESP_ERR_INVALID_ARG;

    ctx->handle = handle;
    ctx->cfg = *cfg;

    if (cfg->sample_rate_hz <= 0 || cfg->sample_rate_hz > 1000000 || cfg->cycles <= 0 || cfg->burden_ohms <= 0.0f || cfg->ct_ratio <= 0.0f || cfg->multisample <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = cfg->adc_atten,
    };
    
    // Configuramos solo el canal
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, cfg->adc_channel, &config));

    return ESP_OK;
}

esp_err_t zmct103c_read_irms(zmct103c_t *ctx, float *irms) {
    if (ctx == NULL || irms == NULL) return ESP_ERR_INVALID_ARG;

    int total_time_us = ctx->cfg.cycles * (1000000 / AC_LINE_FREQ_HZ);
    int delay_us = 1000000 / ctx->cfg.sample_rate_hz;
    int samples = (delay_us > 0) ? (total_time_us / delay_us) : 0;

    if (samples <= 0 || delay_us <= 0) return ESP_ERR_INVALID_ARG;

    if (samples > s_buffer_len) {
        int *new_buf = realloc(s_buffer, samples * sizeof(int));
        if (!new_buf) return ESP_ERR_NO_MEM;
        s_buffer = new_buf;
        s_buffer_len = samples;
    }
    int *buf = s_buffer;

    long sum = 0;

    // 1. Offset DC
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        int raw_sum = 0;
        for (int m = 0; m < ctx->cfg.multisample; m++) {
            esp_err_t err = adc_oneshot_read(ctx->handle, ctx->cfg.adc_channel, &raw);
            if (err != ESP_OK) return err;
            raw_sum += raw;
        }
        raw = raw_sum / ctx->cfg.multisample;
        buf[i] = raw;
        sum += raw;
        esp_rom_delay_us(delay_us);
    }
    float offset = (float)sum / samples;

    // 2. RMS
    float sum_sq = 0;
    for (int i = 0; i < samples; i++) {
        float val = buf[i] - offset;
        sum_sq += val * val;
    }
    
    float rms_raw = sqrt(sum_sq / samples);
    float v_rms_sensor = (rms_raw * 3.3f) / 4095.0f;
    float i_sec = v_rms_sensor / ctx->cfg.burden_ohms;
    *irms = i_sec * ctx->cfg.ct_ratio;

    return ESP_OK;
}
