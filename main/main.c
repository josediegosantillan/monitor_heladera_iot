/**
 * @file main.c
 * @brief Controlador Heladera IoT - Arquitectura Hub & Spoke
 * @author Arq. Gadd / Diego
 * @version 3.0 (Producción: WiFi Wait + WSS + Relay + Sensors)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_types.h"

// --- DRIVERS Y COMPONENTES PROPIOS ---
#include "esp_adc/adc_oneshot.h"
#include "ac_meter.h"
#include "zmct103c.h"
#include "ds18b20.h"
#include "relay_driver.h"
#include "wifi_portal.h"
#include "mqtt_connector.h"
#include "sdkconfig.h"

static const char *TAG = "HELADERA_MAIN";

// --- DEFINICIONES DE HARDWARE (Mapeo desde Kconfig) ---
#define PIN_FACTORY_RESET   GPIO_NUM_0 
#define PIN_TEMP_HELADERA   ((gpio_num_t)CONFIG_GPIO_SENSOR_HELADERA)
#define PIN_TEMP_TABLERO    ((gpio_num_t)CONFIG_GPIO_SENSOR_TABLERO)
#define PIN_AC_VOLTAGE_GPIO ((gpio_num_t)CONFIG_GPIO_AC_VOLTAGE)
#define PIN_AC_CURRENT_GPIO ((gpio_num_t)CONFIG_GPIO_AC_CURRENT)
// FIX: Nombre correcto de la variable Kconfig
#define PIN_RELE_MAIN       ((gpio_num_t)CONFIG_GPIO_RELAY)

// --- PARAMETROS DE CALIBRACION ---
#define TEMP_COEFF_PPM      CONFIG_TEMP_COEFF_PPM 
#define TEMP_COEFF_PPM_I    120 
#define CURRENT_CAL_FACTOR  0.41f
#define CURRENT_EMA_ALPHA   0.20f

// --- SENTINELS (Valores de error) ---
#define TEMP_INVALID_SENTINEL -999.0f
#define ENERGY_INVALID_SENTINEL -1.0f

// --- ESTRUCTURA DE ESTADO GLOBAL ---
typedef struct {
    float temp_heladera;    
    float temp_tablero;     
    float voltaje_rms;      
    float corriente_rms;    
    float potencia_aparente;
    bool  rele_estado;      
} sistema_estado_t;

// Variables Globales Protegidas
static sistema_estado_t g_estado;
static SemaphoreHandle_t g_mutex;
static relay_handle_t g_relay_handle; 

// ---------------------------------------------------------------------
// TAREA 1: Factory Reset (Seguridad Física)
// ---------------------------------------------------------------------
void vTaskFactoryReset(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_FACTORY_RESET),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        if (gpio_get_level(PIN_FACTORY_RESET) == 0) {
            ESP_LOGW(TAG, "⚠️ Boton RESET detectado. Mantené 5s...");
            int contador = 0;
            while (gpio_get_level(PIN_FACTORY_RESET) == 0 && contador < 50) {
                vTaskDelay(pdMS_TO_TICKS(100));
                contador++;
            }
            if (contador >= 50) {
                ESP_LOGE(TAG, "🚨 FACTORY RESET INICIADO - BORRANDO NVS");
                esp_wifi_stop(); 
                vTaskDelay(pdMS_TO_TICKS(100)); 
                
                esp_err_t err = nvs_flash_erase();
                if (err != ESP_OK) ESP_LOGE(TAG, "Error NVS Erase: %s", esp_err_to_name(err));
                
                err = nvs_flash_init();
                if (err != ESP_OK) ESP_LOGE(TAG, "Error NVS Init: %s", esp_err_to_name(err));
                
                ESP_LOGE(TAG, "Reiniciando sistema...");
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ---------------------------------------------------------------------
// TAREA 2: Sensores Térmicos (DS18B20)
// ---------------------------------------------------------------------
void vTaskTermica(void *pvParameters) {
    ds18b20_t s_heladera;
    ds18b20_t s_tablero;
    
    ds18b20_init(&s_heladera, PIN_TEMP_HELADERA);
    ds18b20_init(&s_tablero, PIN_TEMP_TABLERO);
    
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_estado.temp_heladera = TEMP_INVALID_SENTINEL;
        g_estado.temp_tablero = TEMP_INVALID_SENTINEL;
        xSemaphoreGive(g_mutex);
    }
    
    while (1) {
        float t1 = ds18b20_read_temp(&s_heladera);
        float t2 = ds18b20_read_temp(&s_tablero);

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_estado.temp_heladera = t1; 
            g_estado.temp_tablero = t2;
            xSemaphoreGive(g_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ---------------------------------------------------------------------
// TAREA 3: Metrología Eléctrica (AC Meter / ZMCT103C)
// ---------------------------------------------------------------------
void vTaskEnergia(void *pvParameters) {
    float i_ema = 0.0f; 
    adc_unit_t unit_v = ADC_UNIT_1, unit_i = ADC_UNIT_1;
    adc_channel_t chan_v, chan_i;
    
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PIN_AC_VOLTAGE_GPIO, &unit_v, &chan_v));
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PIN_AC_CURRENT_GPIO, &unit_i, &chan_i));

    if (unit_v != ADC_UNIT_1 || unit_i != ADC_UNIT_1) {
        ESP_LOGE(TAG, "ERROR CRÍTICO: Pines ADC deben ser ADC1. Abortando energía.");
        vTaskDelete(NULL);
    }

    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = unit_v, .ulp_mode = ADC_ULP_MODE_DISABLE };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // FIX: Actualizado a ADC_ATTEN_DB_12 para ESP-IDF v5.x
    ac_meter_cfg_t cfg_v = { .channel = chan_v, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT, .fs_hz = 2000, .window_ms = 200 };
    ESP_ERROR_CHECK(ac_meter_init(adc_handle, &cfg_v));

    zmct103c_t zmct;
    // FIX: Actualizado a ADC_ATTEN_DB_12
    zmct103c_cfg_t cfg_i = { .adc_channel = chan_i, .adc_atten = ADC_ATTEN_DB_12, .burden_ohms = 68.0f, .ct_ratio = 1000.0f, .sample_rate_hz = 2000, .cycles = 10, .multisample = 4 };
    ESP_ERROR_CHECK(zmct103c_init(&zmct, adc_handle, &cfg_i));

    while (1) {
        ac_meter_reading_t lec_v;
        float lec_i = 0.0f;
        esp_err_t err_v = ac_meter_read(&lec_v);
        esp_err_t err_i = zmct103c_read_irms(&zmct, &lec_i);

        if (err_v != ESP_OK || err_i != ESP_OK) {
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_estado.voltaje_rms = ENERGY_INVALID_SENTINEL;
                g_estado.corriente_rms = ENERGY_INVALID_SENTINEL;
                g_estado.potencia_aparente = ENERGY_INVALID_SENTINEL;
                xSemaphoreGive(g_mutex);
            }
            vTaskDelay(pdMS_TO_TICKS(500)); 
            continue;
        }

        float temp_actual = 25.0f;
        bool temp_valida = false;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (g_estado.temp_tablero > -273.0f && g_estado.temp_tablero != TEMP_INVALID_SENTINEL) {
                temp_actual = g_estado.temp_tablero;
                temp_valida = true;
            }
            xSemaphoreGive(g_mutex);
        }

        float delta_t = temp_valida ? (temp_actual - 25.0f) : 0.0f;
        float v_final = lec_v.vline_rms * (1.0f + (TEMP_COEFF_PPM / 1e6f) * delta_t);
        float i_final = lec_i * (1.0f + (TEMP_COEFF_PPM_I / 1e6f) * delta_t) * CURRENT_CAL_FACTOR;

        if (i_ema == 0.0f) i_ema = i_final;
        else i_ema = (CURRENT_EMA_ALPHA * i_final) + ((1.0f - CURRENT_EMA_ALPHA) * i_ema);
        i_final = i_ema;

        if (v_final < 18.0f) v_final = 0.0f; 
        if (i_final < 0.05f) i_final = 0.0f;

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_estado.voltaje_rms = v_final;
            g_estado.corriente_rms = i_final;
            g_estado.potencia_aparente = v_final * i_final;
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------
// TAREA 4: Reporte y Telemetría MQTT
// ---------------------------------------------------------------------
void vTaskReporte(void *pvParameters) {
    char json_payload[200];
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    while(1) {
        sistema_estado_t snapshot;
        bool data_ready = false;

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            snapshot = g_estado;
            data_ready = true;
            xSemaphoreGive(g_mutex);
        }

        if (data_ready) {
            // Logueo Local
            ESP_LOGD(TAG, "Estado -> TempH: %.1f | V: %.1f | A: %.2f", 
                     snapshot.temp_heladera, snapshot.voltaje_rms, snapshot.corriente_rms);

            // Envío MQTT
            if (mqtt_app_is_connected()) {
                snprintf(json_payload, sizeof(json_payload), 
                        "{\"temp_heladera\": %.2f, \"temp_tablero\": %.2f, \"voltaje\": %.1f, \"corriente\": %.2f, \"potencia\": %.1f, \"rele\": %s}",
                        (snapshot.temp_heladera == TEMP_INVALID_SENTINEL) ? 0.0 : snapshot.temp_heladera,
                        (snapshot.temp_tablero == TEMP_INVALID_SENTINEL) ? 0.0 : snapshot.temp_tablero,
                        (snapshot.voltaje_rms == ENERGY_INVALID_SENTINEL) ? 0.0 : snapshot.voltaje_rms,
                        (snapshot.corriente_rms == ENERGY_INVALID_SENTINEL) ? 0.0 : snapshot.corriente_rms,
                        snapshot.potencia_aparente,
                        snapshot.rele_estado ? "true" : "false");

                mqtt_app_publish(MQTT_TOPIC_TELEMETRY, json_payload);
                ESP_LOGI(TAG, "📡 MQTT Enviado: %s", json_payload);
            } else {
                ESP_LOGW(TAG, "⚠️ MQTT Desconectado - Guardando silencio...");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ---------------------------------------------------------------------
// MAIN APPLICATION
// ---------------------------------------------------------------------
void app_main(void) {
    // 1. Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Crear Mutex
    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == NULL) {
        ESP_LOGE(TAG, "Error critico: No se pudo crear Mutex");
        esp_restart();
    }

    // 3. Estado Inicial Seguro
    g_estado.temp_heladera = TEMP_INVALID_SENTINEL;
    g_estado.temp_tablero = TEMP_INVALID_SENTINEL;
    g_estado.voltaje_rms = ENERGY_INVALID_SENTINEL;
    g_estado.rele_estado = false;

    // 4. Inicializar Hardware
    ESP_LOGI(TAG, "Iniciando Hardware...");
    
    relay_handle_t r_cfg = { .pin = PIN_RELE_MAIN, .active_high = true }; 
    g_relay_handle = r_cfg;
    relay_init(&g_relay_handle, PIN_RELE_MAIN);
    relay_off(&g_relay_handle); 

    // 5. Iniciar Conectividad
    ESP_LOGI(TAG, "Iniciando WiFi Portal...");
    wifi_portal_init();
    
    // --- FIX: ESPERA ACTIVA DE IP ANTES DE ARRANCAR MQTT ---
    ESP_LOGI(TAG, "⏳ Esperando conexión WiFi y dirección IP...");
    int retry = 0;
    const int max_retries = 30; // 30 segundos de paciencia
    
    while (retry < max_retries) {
        esp_netif_ip_info_t ip_info;
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

        if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            if (ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "✅ ¡WiFi Conectado! IP: " IPSTR, IP2STR(&ip_info.ip));
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
        if (retry % 5 == 0) ESP_LOGI(TAG, "Todavía esperando IP... (%d/%d)", retry, max_retries);
    }

    if (retry >= max_retries) {
        ESP_LOGE(TAG, "❌ Timeout esperando WiFi. Reiniciando para reintentar...");
        esp_restart();
    }
    // -------------------------------------------------------

    // 6. Iniciar MQTT (Ahora seguro porque tenemos red)
    ESP_LOGI(TAG, "Iniciando Stack MQTT...");
    mqtt_app_start();

    // 7. Lanzar Tareas
    xTaskCreate(vTaskFactoryReset, "Task_Reset", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(vTaskTermica, "Task_Clima", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnergia, "Task_Metrologia", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(vTaskReporte, "Task_Reporte", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "=== SISTEMA ARRANCADO Y OPERATIVO ===");
}
