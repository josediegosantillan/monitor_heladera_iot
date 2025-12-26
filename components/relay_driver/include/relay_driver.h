#pragma once
#include "driver/gpio.h"
#include "esp_err.h"

// Definimos lógica positiva: 1 = Relé Activado (Carga ON), 0 = Relé Apagado (Carga OFF)
// Si usaras un módulo de relé optoacoplado que activa con bajo, cambiarías esto.

typedef struct {
    gpio_num_t pin;
    bool active_high; // true: 1 activa el rele. false: 0 activa.
} relay_handle_t;

/**
 * @brief Inicializa el GPIO del relé
 */
esp_err_t relay_init(relay_handle_t *relay, gpio_num_t pin);

/**
 * @brief Activa la carga (Cierra el relé)
 */
esp_err_t relay_on(relay_handle_t *relay);

/**
 * @brief Desactiva la carga (Abre el relé)
 */
esp_err_t relay_off(relay_handle_t *relay);
