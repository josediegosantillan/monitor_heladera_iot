#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_TEMP_ERROR -999.0f

typedef struct {
    gpio_num_t pin;
} ds18b20_t;

/**
 * @brief Inicializa el pin del sensor (Open Drain).
 */
void ds18b20_init(ds18b20_t *sensor, gpio_num_t pin);

/**
 * @brief Lee la temperatura. Tarda aprox 750ms (bloqueante o con delay).
 * @return float Temperatura en Celsius o SENSOR_TEMP_ERROR si falla.
 */
float ds18b20_read_temp(ds18b20_t *sensor);

#ifdef __cplusplus
}
#endif
