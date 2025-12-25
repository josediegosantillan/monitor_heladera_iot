/**
 * @file main.c
 * @brief Controlador Heladera IoT - Versión Final Integrada
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

// Configuración del Proyecto (Kconfig)
#include "sdkconfig.h"

static const char *TAG = "HELADERA_IOT";

// ============================================================
// ⚙️ CONFIGURACIÓN DE PINES Y CANALES (HARDWARE MAP)
// ============================================================
// Si no usaste el menuconfig, ajustá estos valores acá:

// Sensores de Temperatura (GPIOs con salida, NO usar 34-39)
#define PIN_TEMP_HELADERA   GPIO_NUM_32
#define PIN_TEMP_TABLERO    GPIO_NUM_33

// Sensores Eléctricos (ADC1)
// Nota: En ESP32 Clásico:
// GPIO 35 = ADC1_CHANNEL_7 (Sensor VP / ZMPT101B)
// GPIO 34 = ADC1_CHANNEL_6 (Sensor VN / ZMCT103C)

#define ADC_CHAN_VOLTAJE    ADC_CHANNEL_7  // GPIO 35
#define ADC_CHAN_CORRIENTE  ADC_CHANNEL_6  // GPIO 34

// Calibración Térmica
#define TEMP_COEFF_PPM      100 // ppm/°C (Ajuste fino)

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
// 🌡️ TAREA 1: CLIMATIZACIÓN (Sensores DS18B20)
// ============================================================
void vTaskTermica(void *pvParameters) {
    ds18b20_t s_heladera;
    ds18b20_t s_tablero;

    // Inicializamos sensores
    ds18b20_init(&s_heladera, PIN_TEMP_HELADERA);
    ds18b20_init(&s_tablero, PIN_TEMP_TABLERO);

    ESP_LOGI(TAG, "🌡️ Servicio de Temperatura iniciado");

    while (1) {
        // Leemos (tarda ~750ms c/u pero el driver maneja los delays)
        float t1 = ds18b20_read_temp(&s_heladera);
        float t2 = ds18b20_read_temp(&s_tablero);

        // Guardamos en zona segura
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Validamos errores para no ensuciar el struct con -999
            if (t1 != SENSOR_TEMP_ERROR) g_estado.temp_heladera = t1;
            if (t2 != SENSOR_TEMP_ERROR) g_estado.temp_tablero = t2;
            xSemaphoreGive(g_mutex);
        }

        // Muestreo lento: temperatura no cambia tan rápido
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============================================================
// ⚡ TAREA 2: METROLOGÍA (AC Voltaje + Corriente)
// ============================================================
void vTaskEnergia(void *pvParameters) {
    
    // 1. CREACIÓN DEL RECURSO ADC COMPARTIDO (Singleton)
    // --------------------------------------------------
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1, // ESP32 Clásico: GPIOs 32-39 están acá
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    ESP_LOGI(TAG, "⚡ ADC Unit 1 Inicializada y Compartida");

    // 2. CONFIGURACIÓN SENSOR VOLTAJE (ZMPT101B)
    // --------------------------------------------------
    ac_meter_cfg_t cfg_v = {
        .channel = ADC_CHAN_VOLTAJE, 
        .atten = ADC_ATTEN_DB_11,     // <--- CORREGIDO: Máximo para ESP32 Clásico
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .fs_hz = 2000,                // 40 muestras por ciclo de 50Hz
        .window_ms = 200              // 10 ciclos de red para promediar
    };
    // Le pasamos el handle creado arriba
    ESP_ERROR_CHECK(ac_meter_init(adc_handle, &cfg_v));

    // 3. CONFIGURACIÓN SENSOR CORRIENTE (ZMCT103C)
    // --------------------------------------------------
    zmct103c_t zmct;
    zmct103c_cfg_t cfg_i = {
        .adc_channel = ADC_CHAN_CORRIENTE,
        .adc_atten = ADC_ATTEN_DB_11, // <--- CORREGIDO
        .burden_ohms = 68.0f,         // Resistencia de carga
        .ct_ratio = 1000.0f,          // Relación del trafo
        .sample_rate_hz = 2000,
        .cycles = 10,
        .multisample = 4              // Oversampling por software
    };
    // Le pasamos el handle creado arriba
    ESP_ERROR_CHECK(zmct103c_init(&zmct, adc_handle, &cfg_i));

    while (1) {
        ac_meter_reading_t lec_v;
        float lec_i = 0.0f;
        float temp_ref = 25.0f; // Temp de calibración laboratorio

        // A. Lectura Cruda
        ac_meter_read(&lec_v);            // Lee voltaje
        zmct103c_read_irms(&zmct, &lec_i);// Lee corriente

        // B. Obtener Temperatura del Tablero (para compensar)
        float temp_actual = 25.0f;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Si el sensor falla (-999), usamos 25°C para no romper la matemática
            if (g_estado.temp_tablero > -50.0f) {
                temp_actual = g_estado.temp_tablero;
            }
            xSemaphoreGive(g_mutex);
        }

        // C. Compensación Térmica Lineal
        // Si hace calor, la resistencia sube -> V medido cae -> Factor > 1 sube el resultado.
        float delta_t = temp_actual - temp_ref;
        float factor_corr = 1.0f + ((float)TEMP_COEFF_PPM / 1000000.0f) * delta_t;
        
        // Aplicamos corrección
        float v_final = lec_v.vline_rms * factor_corr;

        // D. Zona Muerta (Noise Gate) - "El Fantasma de los 2.5V"
        // Si hay menos de 9V (ruido), forzamos a 0.
        if (v_final < 9.0f) v_final = 0.0f;
        if (lec_i < 0.05f) lec_i = 0.0f;    // Menos de 50mA es ruido

        // E. Actualizar Estado Global
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_estado.voltaje_rms = v_final;
            g_estado.corriente_rms = lec_i;
            g_estado.potencia_aparente = v_final * lec_i;
            xSemaphoreGive(g_mutex);
        }

        // Frecuencia de actualización de mediciones
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================
// 📟 TAREA 3: REPORTE (Monitor Serial)
// ============================================================
void vTaskReporte(void *pvParameters) {
    // Esperamos un poco al arranque para que se estabilicen los sensores
    vTaskDelay(pdMS_TO_TICKS(2000));

    while(1) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Formato limpio tipo tablero industrial
            ESP_LOGW(TAG, "========================================");
            ESP_LOGI(TAG, "❄️  HELADERA: %5.1f °C  | 🌡️  TABLERO: %5.1f °C", 
                     g_estado.temp_heladera, g_estado.temp_tablero);
            
            ESP_LOGI(TAG, "⚡  TENSION:  %5.1f V   | 🔌  CORRIENTE: %4.2f A", 
                     g_estado.voltaje_rms, g_estado.corriente_rms);
            
            ESP_LOGI(TAG, "💡  POTENCIA: %5.1f VA", g_estado.potencia_aparente);
            ESP_LOGW(TAG, "========================================");
            
            xSemaphoreGive(g_mutex);
        }
        
        // Actualizamos pantalla cada 5 segundos (Modo Calmado)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================
// 🚀 APP MAIN (Punto de Entrada)
// ============================================================
void app_main(void) {
    // 1. Inicializar Memoria No Volátil (NVS) - Requerido por WiFi/Phy
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Configurar Logs (Silenciar ruido de drivers)
    esp_log_level_set("*", ESP_LOG_INFO);        // Default: Info
    esp_log_level_set("AC_METER", ESP_LOG_WARN); // Solo errores graves del voltímetro
    esp_log_level_set("ZMCT103C", ESP_LOG_WARN); // Solo errores graves del amperímetro
    esp_log_level_set("DS18B20", ESP_LOG_WARN);  // Solo errores graves de temperatura

    // 3. Crear Mutex para proteger datos compartidos
    g_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "🚀 Iniciando Sistema de Monitoreo Industrial...");

    // 4. Lanzar Tareas (Prioridades y Stack)
    // Tarea Termica: Baja prioridad (0), stack normal.
    xTaskCreatePinnedToCore(vTaskTermica, "Task_Clima", 4096, NULL, 5, NULL, 0);
    
    // Tarea Energia: Alta prioridad (1), stack robusto (usa float/math).
    xTaskCreatePinnedToCore(vTaskEnergia, "Task_Metrologia", 4096, NULL, 10, NULL, 1);
    
    // Tarea Reporte: Baja prioridad, solo imprime.
    xTaskCreatePinnedToCore(vTaskReporte, "Task_Reporte", 3072, NULL, 1, NULL, 0);
}