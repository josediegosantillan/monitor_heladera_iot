#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // adc_unit_t adc_unit; <-- SE ELIMINA
    adc_channel_t adc_channel;
    adc_atten_t adc_atten;
    float burden_ohms;
    float ct_ratio; 
    int sample_rate_hz;
    int cycles;
    int multisample;
} zmct103c_cfg_t;

typedef struct {
    adc_oneshot_unit_handle_t handle; // Referencia guardada aquí
    zmct103c_cfg_t cfg;
} zmct103c_t;

esp_err_t zmct103c_init(zmct103c_t *ctx, adc_oneshot_unit_handle_t handle, const zmct103c_cfg_t *cfg);
esp_err_t zmct103c_read_irms(zmct103c_t *ctx, float *irms);

#ifdef __cplusplus
}
#endif
