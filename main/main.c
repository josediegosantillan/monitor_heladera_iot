/**
 * @file main.c
 * @brief Controlador Heladera IoT con Proteccion de Tension
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h" // Necesario para tiempos absolutos
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

// Componentes Propios
#include "ac_meter.h"
#include "zmct103c.h"
#include "ds18b20.h"
#include "relay_driver.h" 

#include "sdkconfig.h"

static const char *TAG = "HELADERA_IOT";

// ============================================================
// ⚙️ HARDWARE MAP & CONFIG
// ============================================================
#define PIN_TEMP_HELADERA   ((gpio_num_t)CONFIG_GPIO_SENSOR_HELADERA)
#define PIN_TEMP_TABLERO    ((gpio_num_t)CONFIG_GPIO_SENSOR_TABLERO)
#define PIN_AC_VOLTAGE      ((gpio_num_t)CONFIG_GPIO_AC_VOLTAGE)
#define PIN_AC_CURRENT      ((gpio_num_t)CONFIG_GPIO_AC_CURRENT)
#define PIN_RELAY_CTRL      ((gpio_num_t)CONFIG_GPIO_RELAY)

// Umbrales de Protección
#define V_LOW_LIMIT         CONFIG_VOLT_LOW_LIMIT
#define V_HIGH_LIMIT        CONFIG_VOLT_HIGH_LIMIT
#define RECONNECT_DELAY_SEC CONFIG_RECONNECT_DELAY_S

// Calibración
#define TEMP_COEFF_PPM      CONFIG_TEMP_COEFF_PPM
#define CURRENT_CAL_FACTOR  0.41f
#define CURRENT_EMA_ALPHA   0.20f

// ============================================================
// 📊 ESTADOS DEL SISTEMA
// ============================================================
typedef enum {
    ESTADO_INICIALIZANDO,
    ESTADO_NORMAL,      // Tensión OK, Relé Cerrado
    ESTADO_CORTE,       // Tensión Mal, Relé Abierto
    ESTADO_ESPERA       // Tensión volvió a OK, esperando temporizador
} proteccion_state_t;

typedef struct {
    float temp_heladera;
    float temp_tablero;
    float voltaje_rms;
    float corriente_rms;
    float potencia_aparente;
    proteccion_state_t estado_proteccion;
    int64_t tiempo_restante_espera; // Segundos para reconectar
} sistema_estado_t;

static sistema_estado_t g_estado;
static SemaphoreHandle_t g_mutex;
static relay_handle_t g_relay; // Handler del relé

// ============================================================
// 🌡️ TAREA 1: CLIMATIZACIÓN (Solo monitoreo)
// ============================================================
void vTaskTermica(void *pvParameters) {
    ds18b20_t s_heladera, s_tablero;
    ds18b20_init(&s_heladera, PIN_TEMP_HELADERA);
    ds18b20_init(&s_tablero, PIN_TEMP_TABLERO);
    
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
// ⚡ TAREA 2: ENERGÍA Y PROTECCIÓN (Lógica Principal)
// ============================================================
void vTaskEnergia(void *pvParameters) {
    float i_ema = 0.0f;
    int64_t timestamp_corte = 0; // Para medir el tiempo de espera
    
    // Inicializar Relé
    relay_init(&g_relay, PIN_RELAY_CTRL);

    // Inicializar ADC (Código resumido por brevedad, igual al anterior)
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    ac_meter_cfg_t cfg_v = {
        .channel = ADC_CHANNEL_7, // Ojo: Revisar mapeo GPIO35 -> CH7
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .fs_hz = 2000, .window_ms = 200
    };
    // Mapeo dinámico para robustez
    adc_unit_t unit; adc_channel_t chan;
    adc_oneshot_io_to_channel(PIN_AC_VOLTAGE, &unit, &cfg_v.channel);
    ESP_ERROR_CHECK(ac_meter_init(adc_handle, &cfg_v));

    zmct103c_t zmct;
    zmct103c_cfg_t cfg_i = {
        .adc_channel = ADC_CHANNEL_6, // Placeholder, se sobrescribe abajo
        .adc_atten = ADC_ATTEN_DB_11, .burden_ohms = 68.0f, .ct_ratio = 1000.0f,
        .sample_rate_hz = 2000, .cycles = 10, .multisample = 4
    };
    adc_oneshot_io_to_channel(PIN_AC_CURRENT, &unit, &cfg_i.adc_channel);
    ESP_ERROR_CHECK(zmct103c_init(&zmct, adc_handle, &cfg_i));

    // Estado inicial
    proteccion_state_t fsm_state = ESTADO_INICIALIZANDO;
    
    // Al arranque, asumimos que debemos esperar por seguridad
    ESP_LOGW(TAG, "🛡️ Inicio: Esperando estabilización de red (%d s)...", RECONNECT_DELAY_SEC);
    fsm_state = ESTADO_ESPERA;
    timestamp_corte = esp_timer_get_time(); 

    while (1) {
        // --- 1. Adquisición ---
        ac_meter_reading_t lec_v;
        float lec_i = 0.0f;
        ac_meter_read(&lec_v);
        zmct103c_read_irms(&zmct, &lec_i);

        // Compensación Temp (Simplificada para foco en lógica)
        float temp_actual = 25.0f; // Leer de variable global si se quiere
        float factor_corr = 1.0f + ((float)TEMP_COEFF_PPM / 1000000.0f) * (temp_actual - 25.0f);
        
        float v_rms = lec_v.vline_rms * factor_corr;
        if (v_rms < 10.0f) v_rms = 0.0f; // Noise gate

        // Filtro EMA corriente
        lec_i *= CURRENT_CAL_FACTOR; // Ajuste
        i_ema = (CURRENT_EMA_ALPHA * lec_i) + ((1.0f - CURRENT_EMA_ALPHA) * i_ema);
        if (i_ema < 0.05f) i_ema = 0.0f;

        // --- 2. Lógica de Protección (Máquina de Estados) ---
        bool voltage_ok = (v_rms >= V_LOW_LIMIT && v_rms <= V_HIGH_LIMIT);

        switch (fsm_state) {
            case ESTADO_NORMAL:
                // Estamos entregando energía. Chequeamos si hay falla.
                if (!voltage_ok) {
                    ESP_LOGE(TAG, "⛔ CORTE POR TENSION: %.1fV (Rango: %d-%d)", v_rms, V_LOW_LIMIT, V_HIGH_LIMIT);
                    relay_off(&g_relay);
                    fsm_state = ESTADO_CORTE;
                }
                break;

            case ESTADO_CORTE:
                // Estamos cortados. Chequeamos si la tensión volvió.
                if (voltage_ok) {
                    ESP_LOGI(TAG, "✅ Tensión normalizada (%.1fV). Iniciando temporizador de espera.", v_rms);
                    timestamp_corte = esp_timer_get_time(); // Marca de tiempo actual en microsegundos
                    fsm_state = ESTADO_ESPERA;
                }
                break;

            case ESTADO_ESPERA:
                // La tensión está bien, pero esperamos que el gas de la heladera se asiente.
                if (!voltage_ok) {
                    // Si la tensión volvió a fallar durante la espera, volvemos a corte sin resetear timer (o reseteando, depende criterio)
                    ESP_LOGW(TAG, "⚠️ Tensión inestable durante espera (%.1fV). Volviendo a Corte.", v_rms);
                    fsm_state = ESTADO_CORTE;
                } else {
                    int64_t now = esp_timer_get_time();
                    int64_t diff_us = now - timestamp_corte;
                    int64_t diff_sec = diff_us / 1000000LL;

                    if (diff_sec >= RECONNECT_DELAY_SEC) {
                        ESP_LOGI(TAG, "❄️ Tiempo de espera cumplido. ENCENDIENDO HELADERA.");
                        relay_on(&g_relay);
                        fsm_state = ESTADO_NORMAL;
                    }
                }
                break;
            
            default:
                fsm_state = ESTADO_CORTE;
                break;
        }

        // --- 3. Actualizar Globales ---
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_estado.voltaje_rms = v_rms;
            g_estado.corriente_rms = i_ema;
            g_estado.potencia_aparente = v_rms * i_ema;
            g_estado.estado_proteccion = fsm_state;
            
            // Calculamos tiempo restante para mostrar en pantalla
            if (fsm_state == ESTADO_ESPERA) {
                int64_t now = esp_timer_get_time();
                int64_t diff_sec = (now - timestamp_corte) / 1000000LL;
                g_estado.tiempo_restante_espera = RECONNECT_DELAY_SEC - diff_sec;
            } else {
                g_estado.tiempo_restante_espera = 0;
            }
            xSemaphoreGive(g_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(250)); // Muestreo cada 250ms
    }
}

// ============================================================
// 📟 TAREA 3: REPORTE
// ============================================================
void vTaskReporte(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    while(1) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGW(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "V: %5.1fV | I: %4.2fA | P: %5.1fVA", 
                     g_estado.voltaje_rms, g_estado.corriente_rms, g_estado.potencia_aparente);
            
            const char* str_estado = "DESCONOCIDO";
            switch(g_estado.estado_proteccion) {
                case ESTADO_NORMAL: str_estado = "NORMAL (ON)"; break;
                case ESTADO_CORTE:  str_estado = "CORTE (OFF)"; break;
                case ESTADO_ESPERA: str_estado = "ESPERA (OFF)"; break;
                default: break;
            }
            
            if (g_estado.estado_proteccion == ESTADO_ESPERA) {
                ESP_LOGW(TAG, "ESTADO: %s | Reconexion en: %lld s", str_estado, g_estado.tiempo_restante_espera);
            } else {
                ESP_LOGI(TAG, "ESTADO: %s", str_estado);
            }
            ESP_LOGW(TAG, "----------------------------------------");
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============================================================
// MAIN
// ============================================================
void app_main(void) {
    nvs_flash_init();
    esp_log_level_set("*", ESP_LOG_INFO);
    g_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "🚀 Monitor IoT + Protector de Tensión Iniciado");

    xTaskCreatePinnedToCore(vTaskTermica, "Task_Clima", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnergia, "Task_Proteccion", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(vTaskReporte, "Task_Reporte", 3072, NULL, 1, NULL, 0);
}
