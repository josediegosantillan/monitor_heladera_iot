#pragma once
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_oneshot_unit_handle_t adc_handle; // Handle Compartido
    adc_unit_t unit_id;                   // ID para calibración
    adc_channel_t channel;
    float ct_ratio;
    float burden_ohms;
} zmct103c_cfg_t;

typedef struct {
    zmct103c_cfg_t cfg;
    adc_cali_handle_t cali_handle;
    bool cali_enable;
} zmct103c_t;

esp_err_t zmct103c_init(zmct103c_t *ctx, zmct103c_cfg_t *cfg);
esp_err_t zmct103c_read_rms(zmct103c_t *ctx, float *irms_out);

#ifdef __cplusplus
}
#endif
