#include "zmct103c.h"
#include <math.h>
#include "esp_log.h"
#include "esp_rom_sys.h"

esp_err_t zmct103c_init(zmct103c_t *ctx, adc_oneshot_unit_handle_t handle, const zmct103c_cfg_t *cfg) {
    if (!ctx || !handle || !cfg) return ESP_ERR_INVALID_ARG;

    ctx->handle = handle;
    ctx->cfg = *cfg;

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = cfg->adc_atten,
    };
    
    // Configuramos solo el canal
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, cfg->adc_channel, &config));

    return ESP_OK;
}

esp_err_t zmct103c_read_irms(zmct103c_t *ctx, float *irms) {
    int total_time_us = ctx->cfg.cycles * 20000; 
    int delay_us = 1000000 / ctx->cfg.sample_rate_hz;
    int samples = total_time_us / delay_us;

    long sum = 0;
    int *buf = malloc(samples * sizeof(int));
    if (!buf) return ESP_ERR_NO_MEM;

    // 1. Offset DC
    for (int i = 0; i < samples; i++) {
        int raw;
        adc_oneshot_read(ctx->handle, ctx->cfg.adc_channel, &raw);
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
    free(buf);

    float rms_raw = sqrt(sum_sq / samples);
    float v_rms_sensor = (rms_raw * 3.3f) / 4095.0f;
    float i_sec = v_rms_sensor / ctx->cfg.burden_ohms;
    *irms = i_sec * ctx->cfg.ct_ratio;

    return ESP_OK;
}
