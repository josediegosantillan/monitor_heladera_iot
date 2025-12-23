#pragma once
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_oneshot_unit_handle_t adc_handle; // Handle compartido (Inyección de dependencia)
    adc_unit_t    unit_id;                // Necesario solo para iniciar la calibración
    adc_channel_t channel;
    adc_atten_t   atten;
    adc_bitwidth_t bitwidth;
    int           fs_hz;
    int           window_ms;
} ac_meter_cfg_t;

typedef struct {
    float vline_rms;
    float offset_mv;
} ac_meter_reading_t;

esp_err_t ac_meter_init(const ac_meter_cfg_t *cfg);
esp_err_t ac_meter_read(ac_meter_reading_t *out);
void ac_meter_set_k(float k);

#ifdef __cplusplus
}
#endif
