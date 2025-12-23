#include "ds18b20.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void write_bit(gpio_num_t pin, int bit) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(bit ? 6 : 60);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(bit ? 64 : 10);
}

static int read_bit(gpio_num_t pin) {
    int bit = 0;
    gpio_set_level(pin, 0);
    esp_rom_delay_us(6);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(9);
    bit = gpio_get_level(pin);
    esp_rom_delay_us(55);
    return bit;
}

static void write_byte(gpio_num_t pin, uint8_t data) {
    for (int i=0; i<8; i++) write_bit(pin, (data >> i) & 1);
}

static uint8_t read_byte(gpio_num_t pin) {
    uint8_t data = 0;
    for (int i=0; i<8; i++) data |= (read_bit(pin) << i);
    return data;
}

static bool reset(gpio_num_t pin) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(480);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(60);
    bool pres = !gpio_get_level(pin);
    esp_rom_delay_us(420);
    return pres;
}

void ds18b20_init(ds18b20_t *sensor, gpio_num_t pin) {
    sensor->pin = pin;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);
}

float ds18b20_read_temp(ds18b20_t *sensor) {
    if (!reset(sensor->pin)) return -127.0;
    write_byte(sensor->pin, 0xCC);
    write_byte(sensor->pin, 0x44);
    vTaskDelay(pdMS_TO_TICKS(750)); 
    if (!reset(sensor->pin)) return -127.0;
    write_byte(sensor->pin, 0xCC);
    write_byte(sensor->pin, 0xBE);
    uint8_t lsb = read_byte(sensor->pin);
    uint8_t msb = read_byte(sensor->pin);
    int16_t raw = (msb << 8) | lsb;
    return (float)raw / 16.0;
}