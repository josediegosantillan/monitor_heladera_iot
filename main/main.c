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
#include "nvs.h"
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
#include "cJSON.h"

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

// --- PARAMETROS DE PROTECCION ---
#define VOLT_LOW_LIMIT_V    ((float)CONFIG_VOLT_LOW_LIMIT)
#define VOLT_HIGH_LIMIT_V   ((float)CONFIG_VOLT_HIGH_LIMIT)
#define VOLT_HYSTERESIS_V   ((float)CONFIG_VOLT_HYSTERESIS_V)
#define RECONNECT_STABLE_TICKS pdMS_TO_TICKS(CONFIG_RECONNECT_STABLE_S * 1000)

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
    int   t_restante_s;
} sistema_estado_t;

// Variables Globales Protegidas
static sistema_estado_t g_estado;
static SemaphoreHandle_t g_mutex;
static relay_handle_t g_relay_handle; 

// --- CONFIG REMOTA (NVS) ---
typedef struct {
    int vmin;
    int vmax;
    int delay_s;
} heladera_cfg_t;

static heladera_cfg_t g_cfg;

static void cfg_load_defaults(heladera_cfg_t *cfg) {
    cfg->vmin = CONFIG_VOLT_LOW_LIMIT;
    cfg->vmax = CONFIG_VOLT_HIGH_LIMIT;
    cfg->delay_s = CONFIG_RECONNECT_DELAY_S;
}

static void cfg_load_nvs(heladera_cfg_t *cfg) {
    nvs_handle_t h;
    if (nvs_open("heladera_cfg", NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 0;
        if (nvs_get_i32(h, "vmin", &v) == ESP_OK) cfg->vmin = (int)v;
        if (nvs_get_i32(h, "vmax", &v) == ESP_OK) cfg->vmax = (int)v;
        if (nvs_get_i32(h, "delay_s", &v) == ESP_OK) cfg->delay_s = (int)v;
        nvs_close(h);
    }
}

static void cfg_save_nvs(const heladera_cfg_t *cfg) {
    nvs_handle_t h;
    if (nvs_open("heladera_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "vmin", cfg->vmin);
        nvs_set_i32(h, "vmax", cfg->vmax);
        nvs_set_i32(h, "delay_s", cfg->delay_s);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void mqtt_on_message(const char *topic, int topic_len,
                            const char *data, int data_len) {
    const char *cfg_topic = MQTT_TOPIC_CONFIG;
    size_t cfg_len = strlen(cfg_topic);

    if (!topic || !data || topic_len != (int)cfg_len ||
        strncmp(topic, cfg_topic, cfg_len) != 0) {
        return;
    }

    if (data_len <= 0 || data_len >= 128) return;

    char buf[128];
    memcpy(buf, data, data_len);
    buf[data_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) return;

    cJSON *vmin = cJSON_GetObjectItem(root, "vmin");
    cJSON *vmax = cJSON_GetObjectItem(root, "vmax");
    cJSON *delay = cJSON_GetObjectItem(root, "delay_s");
    if (!delay) delay = cJSON_GetObjectItem(root, "delay");

    if (cJSON_IsNumber(vmin) && cJSON_IsNumber(vmax) && cJSON_IsNumber(delay)) {
        int nvmin = (int)vmin->valuedouble;
        int nvmax = (int)vmax->valuedouble;
        int ndelay = (int)delay->valuedouble;

        if (nvmin > 0 && nvmax > 0 && ndelay > 0 && nvmin < nvmax) {
            heladera_cfg_t next = { .vmin = nvmin, .vmax = nvmax, .delay_s = ndelay };
            bool applied = false;

            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_cfg = next;
                xSemaphoreGive(g_mutex);
                applied = true;
            }
            if (applied) {
                cfg_save_nvs(&next);
            }
        }
    }
    cJSON_Delete(root);
}

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
    char json_payload[240];
    
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
                        "{\"temp_heladera\": %.2f, \"temp_tablero\": %.2f, \"voltaje\": %.1f, \"corriente\": %.2f, \"potencia\": %.1f, \"rele\": %s, \"t_restante\": %d}",
                        (snapshot.temp_heladera == TEMP_INVALID_SENTINEL) ? 0.0 : snapshot.temp_heladera,
                        (snapshot.temp_tablero == TEMP_INVALID_SENTINEL) ? 0.0 : snapshot.temp_tablero,
                        (snapshot.voltaje_rms == ENERGY_INVALID_SENTINEL) ? 0.0 : snapshot.voltaje_rms,
                        (snapshot.corriente_rms == ENERGY_INVALID_SENTINEL) ? 0.0 : snapshot.corriente_rms,
                        snapshot.potencia_aparente,
                        snapshot.rele_estado ? "true" : "false",
                        snapshot.t_restante_s);

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
// TAREA 5: Proteccion por Tension (Corte y Reconexion)
// ---------------------------------------------------------------------
void vTaskProteccion(void *pvParameters) {
    const TickType_t check_delay = pdMS_TO_TICKS(200);
    const TickType_t stable_window = RECONNECT_STABLE_TICKS;
    bool cut_active = true; // Treat power-up as a cut; wait after voltage returns
    bool stable_timing = false;
    bool delay_timing = false;
    TickType_t reconnect_start = 0;
    TickType_t stable_start = 0;

    while (1) {
        float v = ENERGY_INVALID_SENTINEL;
        bool relay_state = false;
        int t_restante_s = 0;

        heladera_cfg_t cfg = {
            .vmin = CONFIG_VOLT_LOW_LIMIT,
            .vmax = CONFIG_VOLT_HIGH_LIMIT,
            .delay_s = CONFIG_RECONNECT_DELAY_S
        };

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            v = g_estado.voltaje_rms;
            relay_state = g_estado.rele_estado;
            cfg = g_cfg;
            xSemaphoreGive(g_mutex);
        }

        if (cfg.vmin <= 0 || cfg.vmax <= 0 || cfg.vmin >= cfg.vmax) {
            cfg.vmin = CONFIG_VOLT_LOW_LIMIT;
            cfg.vmax = CONFIG_VOLT_HIGH_LIMIT;
        }
        if (cfg.delay_s <= 0) {
            cfg.delay_s = CONFIG_RECONNECT_DELAY_S;
        }

        float low_limit = (float)cfg.vmin;
        float high_limit = (float)cfg.vmax;
        TickType_t reconnect_delay = pdMS_TO_TICKS((uint32_t)cfg.delay_s * 1000U);

        bool v_valid = (v != ENERGY_INVALID_SENTINEL) && (v > 1.0f);
        bool must_cut = (!v_valid) || (v < low_limit) || (v > high_limit);

        if (must_cut) {
            cut_active = true;
            delay_timing = false;
            stable_timing = false;

            if (relay_state) {
                relay_off(&g_relay_handle);
                if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    g_estado.rele_estado = false;
                    xSemaphoreGive(g_mutex);
                }
                ESP_LOGW(TAG, "Corte por tension fuera de rango: %.1fV", v);
            }
        } else {
            float low_reconnect = low_limit + VOLT_HYSTERESIS_V;
            float high_reconnect = high_limit - VOLT_HYSTERESIS_V;
            if (low_reconnect >= high_reconnect) {
                low_reconnect = low_limit;
                high_reconnect = high_limit;
            }
            bool in_reconnect_band = (v >= low_reconnect) && (v <= high_reconnect);

            if (cut_active) {
                TickType_t now = xTaskGetTickCount();
                if (in_reconnect_band) {
                    if (!delay_timing) {
                        reconnect_start = now;
                        delay_timing = true;
                    }
                    if (!stable_timing) {
                        stable_start = now;
                        stable_timing = true;
                    }
                    TickType_t elapsed_delay = now - reconnect_start;
                    TickType_t elapsed_stable = now - stable_start;
                    TickType_t rem_delay = (elapsed_delay >= reconnect_delay) ? 0 : (reconnect_delay - elapsed_delay);
                    TickType_t rem_stable = (elapsed_stable >= stable_window) ? 0 : (stable_window - elapsed_stable);
                    TickType_t rem = (rem_delay > rem_stable) ? rem_delay : rem_stable;
                    if (rem > 0) {
                        uint32_t rem_ms = (uint32_t)pdTICKS_TO_MS(rem);
                        t_restante_s = (int)((rem_ms + 999U) / 1000U);
                    }
                    if ((now - reconnect_start) >= reconnect_delay && (now - stable_start) >= stable_window) {
                        relay_on(&g_relay_handle);
                        cut_active = false;
                        delay_timing = false;
                        stable_timing = false;
                        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            g_estado.rele_estado = true;
                            xSemaphoreGive(g_mutex);
                        }
                        ESP_LOGI(TAG, "Reconexion habilitada. Voltaje: %.1fV", v);
                    }
                } else {
                    delay_timing = false;
                    stable_timing = false;
                }
            } else if (!relay_state) {
                relay_on(&g_relay_handle);
                if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    g_estado.rele_estado = true;
                    xSemaphoreGive(g_mutex);
                }
                ESP_LOGI(TAG, "Rele habilitado. Voltaje: %.1fV", v);
            }
        }

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_estado.t_restante_s = t_restante_s;
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(check_delay);
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
    g_estado.t_restante_s = 0;

    cfg_load_defaults(&g_cfg);
    cfg_load_nvs(&g_cfg);

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
    mqtt_app_set_rx_callback(mqtt_on_message);

    mqtt_app_start();

    // 7. Lanzar Tareas
    xTaskCreate(vTaskFactoryReset, "Task_Reset", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(vTaskTermica, "Task_Clima", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnergia, "Task_Metrologia", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(vTaskProteccion, "Task_Proteccion", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(vTaskReporte, "Task_Reporte", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "=== SISTEMA ARRANCADO Y OPERATIVO ===");
}
