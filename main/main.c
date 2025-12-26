/**
 * @file main.c
 * @brief Controlador Heladera IoT - Integración Final
 * @author Ingeniero Senior ESP-IDF
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// ADC API (v5.x)
#include "esp_adc/adc_oneshot.h"

// Componentes Propios
#include "ac_meter.h"
#include "zmct103c.h"
#include "ds18b20.h"
#include "wifi_portal.h"  // <--- El Portal Cautivo Blindado

// Configuración del Proyecto (Kconfig)
#include "sdkconfig.h"

static const char *TAG = "HELADERA_MAIN";

// ============================================================
// ⚙️ CONFIGURACIÓN DE HARDWARE
// ============================================================

// Botón de Reset de Fábrica (Usamos el botón BOOT de la placa)
#define PIN_FACTORY_RESET   GPIO_NUM_0 

// Sensores de Temperatura
#define PIN_TEMP_HELADERA   ((gpio_num_t)CONFIG_GPIO_SENSOR_HELADERA)
#define PIN_TEMP_TABLERO    ((gpio_num_t)CONFIG_GPIO_SENSOR_TABLERO)

// Sensores Eléctricos (ADC1)
#define PIN_AC_VOLTAGE_GPIO ((gpio_num_t)CONFIG_GPIO_AC_VOLTAGE)
#define PIN_AC_CURRENT_GPIO ((gpio_num_t)CONFIG_GPIO_AC_CURRENT)

// Calibración
#define TEMP_COEFF_PPM      CONFIG_TEMP_COEFF_PPM 
#define TEMP_COEFF_PPM_I    120 
#define CURRENT_CAL_FACTOR  0.41f
#define CURRENT_EMA_ALPHA   0.20f

// ============================================================
// 📊 ESTRUCTURA DE DATOS COMPARTIDA
// ============================================================
typedef struct {
    float temp_heladera;    // °C
    float temp_tablero;     // °C
    float voltaje_rms;      // V
    float corriente_rms;    // A
    float potencia_aparente;// VA
} sistema_estado_t;

static sistema_estado_t g_estado;
static SemaphoreHandle_t g_mutex;

// ============================================================
// 🚨 TAREA 0: FACTORY RESET (Botón Físico)
// ============================================================
// Monitorea el botón BOOT. Si se presiona > 5s, borra NVS.
void vTaskFactoryReset(void *pvParameters) {
    // Configurar GPIO 0 como entrada con Pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_FACTORY_RESET),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        // Lógica invertida: 0 es presionado
        if (gpio_get_level(PIN_FACTORY_RESET) == 0) {
            ESP_LOGW(TAG, "⚠️ Boton RESET detectado. Mantené 5s para borrar WiFi...");
            
            int contador = 0;
            // Esperamos 5 segundos (50 * 100ms) chequeando que siga apretado
            while (gpio_get_level(PIN_FACTORY_RESET) == 0 && contador < 50) {
                vTaskDelay(pdMS_TO_TICKS(100));
                contador++;
            }

            if (contador >= 50) {
                ESP_LOGE(TAG, "🚨 FACTORY RESET: BORRANDO CREDENCIALES...");
                
                // Borrado físico de la memoria
                nvs_flash_erase();
                nvs_flash_init(); 
                
                ESP_LOGE(TAG, "Reinicio inminente...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            } else {
                ESP_LOGI(TAG, "Reset cancelado (tiempo insuficiente).");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Polling lento para no comer CPU
    }
}

// ============================================================
// 🌡️ TAREA 1: CLIMATIZACIÓN (Sensores DS18B20)
// ============================================================
void vTaskTermica(void *pvParameters) {
    ds18b20_t s_heladera;
    ds18b20_t s_tablero;

    ds18b20_init(&s_heladera, PIN_TEMP_HELADERA);
    ds18b20_init(&s_tablero, PIN_TEMP_TABLERO);
    
    ESP_LOGI(TAG, "🌡️ Servicio de Temperatura iniciado");

    while (1) {
        float t1 = ds18b20_read_temp(&s_heladera);
        float t2 = ds18b20_read_temp(&s_tablero);

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (t1 != SENSOR_TEMP_ERROR) g_estado.temp_heladera = t1;
            if (t2 != SENSOR_TEMP_ERROR) g_estado.temp_tablero = t2;
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============================================================
// ⚡ TAREA 2: METROLOGÍA (AC Voltaje + Corriente)
// ============================================================
void vTaskEnergia(void *pvParameters) {
    float i_ema = 0.0f;
    
    // 1. Configuración ADC Unit 1 (Compartido)
    adc_unit_t unit_v = ADC_UNIT_1;
    adc_unit_t unit_i = ADC_UNIT_1;
    adc_channel_t chan_v, chan_i;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PIN_AC_VOLTAGE_GPIO, &unit_v, &chan_v));
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PIN_AC_CURRENT_GPIO, &unit_i, &chan_i));

    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit_v,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    ESP_LOGI(TAG, "⚡ ADC Unit 1 Inicializada");

    // 2. Sensor Voltaje
    ac_meter_cfg_t cfg_v = {
        .channel = chan_v, 
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .fs_hz = 2000,
        .window_ms = 200
    };
    ESP_ERROR_CHECK(ac_meter_init(adc_handle, &cfg_v));

    // 3. Sensor Corriente
    zmct103c_t zmct;
    zmct103c_cfg_t cfg_i = {
        .adc_channel = chan_i,
        .adc_atten = ADC_ATTEN_DB_11, 
        .burden_ohms = 68.0f,
        .ct_ratio = 1000.0f,
        .sample_rate_hz = 2000,
        .cycles = 10,
        .multisample = 4
    };
    ESP_ERROR_CHECK(zmct103c_init(&zmct, adc_handle, &cfg_i));

    while (1) {
        ac_meter_reading_t lec_v;
        float lec_i = 0.0f;
        float temp_ref = 25.0f;

        // Lecturas
        esp_err_t err_v = ac_meter_read(&lec_v);
        esp_err_t err_i = zmct103c_read_irms(&zmct, &lec_i);

        if (err_v != ESP_OK || err_i != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Obtener Temp Tablero para compensación
        float temp_actual = 25.0f;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (g_estado.temp_tablero > -50.0f) temp_actual = g_estado.temp_tablero;
            xSemaphoreGive(g_mutex);
        }

        // Compensación Térmica
        float delta_t = temp_actual - temp_ref;
        float factor_corr = 1.0f + ((float)TEMP_COEFF_PPM / 1000000.0f) * delta_t;
        float factor_corr_i = 1.0f + ((float)TEMP_COEFF_PPM_I / 1000000.0f) * delta_t;
        
        float v_final = lec_v.vline_rms * factor_corr;
        float i_final = lec_i * factor_corr_i * CURRENT_CAL_FACTOR;

        // Filtro EMA
        if (i_ema == 0.0f) i_ema = i_final;
        else i_ema = (CURRENT_EMA_ALPHA * i_final) + ((1.0f - CURRENT_EMA_ALPHA) * i_ema);
        i_final = i_ema;

        // Zona Muerta
        if (v_final < 9.0f) v_final = 0.0f;
        if (i_final < 0.05f) i_final = 0.0f;

        // Actualizar Global
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_estado.voltaje_rms = v_final;
            g_estado.corriente_rms = i_final;
            g_estado.potencia_aparente = v_final * i_final;
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================
// 📟 TAREA 3: REPORTE (Monitor Serial)
// ============================================================
void vTaskReporte(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    while(1) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGW(TAG, "========================================");
            ESP_LOGI(TAG, "❄️  HELADERA: %5.1f °C  | 🌡️  TABLERO: %5.1f °C", 
                     g_estado.temp_heladera, g_estado.temp_tablero);
            
            ESP_LOGI(TAG, "⚡  TENSION:  %5.1f V   | 🔌  CORRIENTE: %4.2f A", 
                     g_estado.voltaje_rms, g_estado.corriente_rms);
            
            ESP_LOGI(TAG, "💡  POTENCIA: %5.1f VA", g_estado.potencia_aparente);
            ESP_LOGW(TAG, "========================================");
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================
// 🚀 APP MAIN
// ============================================================
void app_main(void) {
    // 1. Init NVS (Crítico para WiFi y Calibración)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Logging Levels
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("AC_METER", ESP_LOG_WARN);
    esp_log_level_set("ZMCT103C", ESP_LOG_WARN);
    esp_log_level_set("DS18B20", ESP_LOG_WARN);

    g_mutex = xSemaphoreCreateMutex();

    // 3. TAREA DE SEGURIDAD (Factory Reset Físico)
    // Se lanza primero para poder borrar datos antes de que arranque el WiFi si es necesario
    xTaskCreate(vTaskFactoryReset, "Task_Reset", 2048, NULL, 1, NULL);

    // 4. CONECTIVIDAD (Portal Cautivo)
    // Intenta conectar. Si falla, levanta AP + Portal.
    ESP_LOGI(TAG, "📡 Iniciando Gestor de Conectividad...");
    wifi_portal_init();

    ESP_LOGI(TAG, "🚀 Iniciando Monitoreo Industrial...");

    // 5. Tareas de Aplicación
    xTaskCreatePinnedToCore(vTaskTermica, "Task_Clima", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnergia, "Task_Metrologia", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(vTaskReporte, "Task_Reporte", 3072, NULL, 1, NULL, 0);
}
