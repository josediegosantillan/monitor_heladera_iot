#pragma once
#include <stdint.h>
#include <stdbool.h>

// Definiciones de Tópicos (Idealmente esto iría en Kconfig, pero hardcodeamos por ahora)
#define MQTT_TOPIC_TELEMETRY "heladera/telemetria"
#define MQTT_TOPIC_STATUS    "heladera/estado"
#define MQTT_TOPIC_CONFIG   "heladera/config"


typedef void (*mqtt_rx_cb_t)(const char *topic, int topic_len,
                             const char *data, int data_len);

void mqtt_app_set_rx_callback(mqtt_rx_cb_t cb);

/**
 * @brief Inicializa el stack MQTT sobre WSS
 */
void mqtt_app_start(void);

/**
 * @brief Publica un mensaje JSON
 * * @param topic Tópico destino
 * @param data String con el payload (JSON)
 * @return true si se encoló correctamente
 */
bool mqtt_app_publish(const char *topic, const char *data);

/**
 * @brief Verifica si estamos conectados al broker
 */
bool mqtt_app_is_connected(void);
