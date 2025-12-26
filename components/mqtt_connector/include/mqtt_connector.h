#pragma once
#include <stdint.h>
#include <stdbool.h>

// Definiciones de Tópicos (Idealmente esto iría en Kconfig, pero hardcodeamos por ahora)
#define MQTT_TOPIC_TELEMETRY "heladera/telemetria"
#define MQTT_TOPIC_STATUS    "heladera/estado"

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
