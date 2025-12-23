#pragma once
#include "driver/gpio.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { gpio_num_t pin; } ds18b20_t;
void ds18b20_init(ds18b20_t *sensor, gpio_num_t pin);
float ds18b20_read_temp(ds18b20_t *sensor);
#ifdef __cplusplus
}
#endif
