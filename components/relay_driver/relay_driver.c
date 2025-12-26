#include "relay_driver.h"
#include "esp_log.h"

static const char *TAG = "RELAY_DRIVER";

esp_err_t relay_init(relay_handle_t *relay, gpio_num_t pin) {
    relay->pin = pin;
    relay->active_high = true; // Para NPN 2N2222 es true.

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err == ESP_OK) {
        // Arrancamos apagado por seguridad
        relay_off(relay);
        ESP_LOGI(TAG, "Rele inicializado en GPIO %d", pin);
    }
    return err;
}

esp_err_t relay_on(relay_handle_t *relay) {
    int level = relay->active_high ? 1 : 0;
    return gpio_set_level(relay->pin, level);
}

esp_err_t relay_off(relay_handle_t *relay) {
    int level = relay->active_high ? 0 : 1;
    return gpio_set_level(relay->pin, level);
}