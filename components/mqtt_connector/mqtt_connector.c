/**
 * @file mqtt_connector.c
 * @brief Cliente MQTT Seguro sobre WSS - Versión Industrial v4 (Race-Condition Free)
 * @author Arq. Gadd / Diego
 * @details Manejo robusto de punteros, validación de eventos zombies y limpieza atómica.
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_err.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "mqtt_connector.h"

static const char *TAG = "MQTT_WSS";

// Cliente Global (Puntero volátil para asegurar visibilidad entre Cores)
static _Atomic esp_mqtt_client_handle_t g_client = ATOMIC_VAR_INIT(NULL);

// Estado de conexión (Atómico para thread-safety)
static atomic_bool is_connected = ATOMIC_VAR_INIT(false);
static mqtt_rx_cb_t s_rx_cb = NULL;

/**
 * @brief Manejador de eventos MQTT
 * @note Se ejecuta en el contexto de la tarea mqtt_task (o sys_evt loop)
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    // FIX #1: Obtener el handle directamente del evento (Fuente de Verdad)
    esp_mqtt_client_handle_t event_client = event->client;

    // FIX #2: Protección contra condiciones de carrera (Zombie Check)
    // Si el evento pertenece a un cliente que ya destruimos (g_client cambió o es NULL), salimos.
    esp_mqtt_client_handle_t current_client = atomic_load(&g_client);
    if (event_client != current_client) {
        ESP_LOGW(TAG, "⚠️ Evento ignorado de cliente obsoleto/zombie (ptr=%p)", event_client);
        return;
    }

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "✅ MQTT Conectado (WSS)");
        atomic_store(&is_connected, true);
        
        // Publicar mensaje de nacimiento
        // Nota: Para suscripciones críticas, mover a MQTT_EVENT_SUBSCRIBED
        esp_mqtt_client_publish(event_client, MQTT_TOPIC_STATUS, "ONLINE", 0, 1, 1);
        esp_mqtt_client_subscribe(event_client, MQTT_TOPIC_CONFIG, 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "⚠️ MQTT Desconectado");
        atomic_store(&is_connected, false);
        break;

    case MQTT_EVENT_DATA:
        if (s_rx_cb && event->topic && event->data) {
            s_rx_cb(event->topic, event->topic_len, event->data, event->data_len);
        }
        break;

    case MQTT_EVENT_ERROR:
        if (event->error_handle == NULL) {
            ESP_LOGE(TAG, "MQTT Error (Contexto desconocido)");
        } else {
            ESP_LOGE(TAG, "MQTT Error type: %d", event->error_handle->error_type);
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "TLS Last Err: %d | Stack Err: %d", 
                         event->error_handle->esp_tls_last_esp_err,
                         event->error_handle->esp_tls_stack_err);
            }
        }
        break;

    default:
        break;
    }
}

void mqtt_app_set_rx_callback(mqtt_rx_cb_t cb) {
    s_rx_cb = cb;
}

void mqtt_app_start(void) {
    // 1. Limpieza de cliente previo (Reset Seguro)
    esp_mqtt_client_handle_t old_client = atomic_exchange(&g_client, NULL); // Seteamos NULL atómicamente primero
    
    if (old_client != NULL) {
        ESP_LOGW(TAG, "Reiniciando cliente MQTT... Deteniendo anterior.");
        // FIX #2: Detener tráfico primero para minimizar eventos pendientes
        esp_mqtt_client_stop(old_client);
        // Destruir recursos
        esp_mqtt_client_destroy(old_client);
    }

    // 2. Configuración
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "wss://thebaltoteam.com.ar/mqtt", 
                .port = 443,
            },
            .verification = {
                .crt_bundle_attach = esp_crt_bundle_attach,
            },
        },
        .credentials = {
            .username = "esp32_heladera",
            .authentication = {
                .password = "291289",
            },
        },
        .session = {
            .keepalive = 30,
            .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
            .last_will = {
                .topic = MQTT_TOPIC_STATUS,
                .msg = "OFFLINE",
                .qos = 1,
                .retain = 1,
            },
            // Deshabilitar autoreconnect automático si queremos controlarlo manualmente
            // .disable_auto_reconnect = false (default)
        },
    };

    ESP_LOGI(TAG, "Inicializando nuevo cliente MQTT...");
    
    // 3. Inicialización Local
    esp_mqtt_client_handle_t new_client = esp_mqtt_client_init(&mqtt_cfg);
    if (new_client == NULL) {
        ESP_LOGE(TAG, "❌ Error crítico: OOM al crear cliente MQTT");
        return;
    }
    
    // 4. Registro de Eventos
    // Pasamos NULL en user_data porque ahora confiamos en event->client
    esp_err_t err = esp_mqtt_client_register_event(new_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error registrando eventos: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(new_client);
        return;
    }
    
    // 5. Asignación Atómica del Global (Habilitamos al handler para reconocerlo)
    atomic_store(&g_client, new_client);

    // 6. Arranque
    err = esp_mqtt_client_start(new_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error arrancando cliente: %s", esp_err_to_name(err));
        // Rollback
        atomic_store(&g_client, NULL);
        esp_mqtt_client_destroy(new_client);
        return;
    }

    ESP_LOGI(TAG, "Cliente MQTT arrancado correctamente");
}

bool mqtt_app_publish(const char *topic, const char *data) {
    if (topic == NULL || data == NULL) return false;

    // Snapshot atómico del cliente actual
    esp_mqtt_client_handle_t client = atomic_load(&g_client);
    bool connected = atomic_load(&is_connected);

    if (client != NULL && connected) {
        int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 0, 0);
        return (msg_id >= 0);
    }
    return false;
}

bool mqtt_app_is_connected(void) {
    return atomic_load(&is_connected);
}
