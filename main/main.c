#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h" // NECESARIO

// Componentes
#include "ac_meter.h"
#include "zmct103c.h"
#include "ds18b20.h"
#include "sdkconfig.h"

static const char *TAG = "HELADERA_IOT";

// ================= ESTRUCTURA DE DATOS GLOBAL =================
typedef struct {
    float temp_heladera;
    float temp_tablero;
    float voltaje_rms;
    float corriente_rms;
    float potencia_aparente;
} sistema_estado_t;

static sistema_estado_t g_estado;
static SemaphoreHandle_t g_mutex; 

// ================= TAREA 1: TEMPERATURA =================
void vTaskTermica(void *pvParameters) {
    ds18b20_t s_heladera;
    ds18b20_t s_tablero;

    ds18b20_init(&s_heladera, (gpio_num_t)CONFIG_GPIO_SENSOR_HELADERA);
    ds18b20_init(&s_tablero, (gpio_num_t)CONFIG_GPIO_SENSOR_TABLERO);

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

// ================= TAREA 2: ELECTRICIDAD (CON ADC COMPARTIDO) =================
void vTaskEnergia(void *pvParameters) {
    
    // 1. CREAR EL RECURSO COMPARTIDO (ADC UNIT)
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1, // Ambos sensores deben estar en ADC1 (GPIOs 32-39)
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    
    // 2. Inicializar Voltaje (Pasando handle)
    ac_meter_cfg_t cfg_v = {
        .channel = (adc_channel_t)6, // GPIO34 (Ejemplo, verificar pinout)
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .fs_hz = 2000,
        .window_ms = 200
    };
    ac_meter_init(adc_handle, &cfg_v);
    
    // 3. Inicializar Corriente (Pasando handle)
    zmct103c_t zmct;
    zmct103c_cfg_t cfg_i = {
        .adc_channel = (adc_channel_t)7, // GPIO35 (Ejemplo, verificar pinout)
        .adc_atten = ADC_ATTEN_DB_12,
        .burden_ohms = 68.0f,
        .ct_ratio = 1000.0f,
        .sample_rate_hz = 2000,
        .cycles = 10,
        .multisample = 4
    };
    zmct103c_init(&zmct, adc_handle, &cfg_i);

    ESP_LOGI(TAG, "Sensores electricos iniciados correctamente.");

    while (1) {
        ac_meter_reading_t lec_v;
        float lec_i = 0.0f;
        float temp_ref = 25.0f;

        // Lectura
        ac_meter_read(&lec_v);
        zmct103c_read_irms(&zmct, &lec_i);

        // Compensación Térmica
        float temp_actual = 25.0f;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            temp_actual = g_estado.temp_tablero;
            xSemaphoreGive(g_mutex);
        }

        // Si la lectura de temperatura es válida (distinta de 0 o error), compensamos
        if (temp_actual < -50.0f) temp_actual = 25.0f; // Fallback simple

        float delta_t = temp_actual - temp_ref;
        float factor_corr = 1.0f + ((float)CONFIG_TEMP_COEFF_PPM / 1000000.0f) * delta_t;

        float v_final = lec_v.vline_rms * factor_corr;
        
        // Guardar datos
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_estado.voltaje_rms = v_final;
            g_estado.corriente_rms = lec_i;
            g_estado.potencia_aparente = v_final * lec_i;
            xSemaphoreGive(g_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ================= TAREA 3: REPORTE =================
void vTaskReporte(void *pvParameters) {
    while(1) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "--- ESTADO DEL SISTEMA ---");
            ESP_LOGI(TAG, "Temp Heladera: %.1f C", g_estado.temp_heladera);
            ESP_LOGI(TAG, "Temp Tablero:  %.1f C", g_estado.temp_tablero);
            ESP_LOGI(TAG, "Red Electrica: %.1f V | %.2f A", g_estado.voltaje_rms, g_estado.corriente_rms);
            ESP_LOGI(TAG, "Potencia:      %.1f VA", g_estado.potencia_aparente);
            ESP_LOGI(TAG, "--------------------------");
            xSemaphoreGive(g_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_mutex = xSemaphoreCreateMutex();

    // Crear Tareas
    xTaskCreatePinnedToCore(vTaskTermica, "Task_Clima", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnergia, "Task_Metrologia", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(vTaskReporte, "Task_Reporte", 2048, NULL, 1, NULL, 0);
}
