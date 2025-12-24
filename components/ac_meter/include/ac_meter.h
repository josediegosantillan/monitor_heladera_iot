#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // adc_unit_t unit;  <-- SE ELIMINA (ya viene en el handle)
    adc_channel_t channel;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    int fs_hz;
    int window_ms;
} ac_meter_cfg_t;

typedef struct {
    float vline_rms;
    float raw_rms;
} ac_meter_reading_t;

// Init recibe el handle del ADC ya creado
esp_err_t ac_meter_init(adc_oneshot_unit_handle_t adc_handle, const ac_meter_cfg_t *cfg);
esp_err_t ac_meter_read(ac_meter_reading_t *out);

#ifdef __cplusplus
}
#endif