/**
 * @file main.c
 * @brief Monitor de Heladera IoT - v3.0 (Thread-Safe & Error Proof)
 */
#include <stdio.h>
#include <math.h> // Para NAN
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // <--- NECESARIO PARA MUTEX
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_check.h"

// Componentes
#include "ds18b20.h"
#include "zmct103c.h"
#include "ac_meter.h"

static const char *TAG = "SYS_MAIN";

// --- CONFIGURACIÓN ---
#define PIN_TEMP_GABINETE   GPIO_NUM_32
#define PIN_TEMP_HELADERA   GPIO_NUM_33
#define PIN_ADC_CORRIENTE   ADC_CHANNEL_6 // GPIO 34
#define PIN_ADC_TENSION     ADC_CHANNEL_7 // GPIO 35
#define LED_STATUS          GPIO_NUM_2

// --- ESTRUCTURA PROTEGIDA ---
typedef struct {
    float t_gabinete; 
    float t_heladera; 
    float v_red;      
    float i_motor;
    bool  sensor_error; // Flag de error general
} sistema_t;

static sistema_t sys_data = {0}; // Protegida por Mutex
static SemaphoreHandle_t sys_mutex = NULL; // El "semáforo" de acceso
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

// --- TAREA TÉRMICA (Optimizada con DelayUntil) ---
void vTaskTermica(void *pv) {
    ds18b20_t s_gab, s_hel;
    ds18b20_init(&s_gab, PIN_TEMP_GABINETE);
    ds18b20_init(&s_hel, PIN_TEMP_HELADERA);

    // Valores iniciales seguros
    if(xSemaphoreTake(sys_mutex, portMAX_DELAY)) {
        sys_data.t_gabinete = 25.0f;
        sys_data.t_heladera = 5.0f;
        xSemaphoreGive(sys_mutex);
    }

    const TickType_t xPeriod = pdMS_TO_TICKS(2000);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Leemos sensores (bloqueante)
        float t1 = ds18b20_read_temp(&s_gab);
        float t2 = ds18b20_read_temp(&s_hel);

        // ZONA CRÍTICA: Actualizar datos
        if (xSemaphoreTake(sys_mutex, pdMS_TO_TICKS(100))) {
            if (t1 > -100) sys_data.t_gabinete = t1;
            if (t2 > -100) sys_data.t_heladera = t2;
            xSemaphoreGive(sys_mutex);
        }

        // Mantiene periodo constante de 2s independiente del tiempo de lectura
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// --- TAREA ELÉCTRICA (Prioridad Baja para no colgar CPU por polling) ---
void vTaskElectrica(void *pv) {
    // 1. Setup Hardware (Singleton Unit)
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    // 2. Setup Tensión
    ac_meter_cfg_t cfg_v = {
        .adc_handle = s_adc_handle,
        .unit_id = ADC_UNIT_1,
        .channel = PIN_ADC_TENSION,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .fs_hz = 2000, .window_ms = 200
    };
    if (ac_meter_init(&cfg_v) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo critico sensor Tension");
        vTaskDelete(NULL);
    }
    ac_meter_set_k(220.0f);

    // 3. Setup Corriente
    zmct103c_cfg_t cfg_i = {
        .adc_handle = s_adc_handle,
        .unit_id = ADC_UNIT_1,
        .channel = PIN_ADC_CORRIENTE,
        .ct_ratio = 1000, .burden_ohms = 68
    };
    zmct103c_t sens_i;
    if (zmct103c_init(&sens_i, &cfg_i) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo critico sensor Corriente");
        vTaskDelete(NULL);
    }

    while(1) {
        ac_meter_reading_t r_v;
        float i_rms = 0;
        
        // Polling de sensores (~400ms total). 
        // Prioridad 1 permite que WiFi/IDLE interrumpan si es necesario.
        esp_err_t err_v = ac_meter_read(&r_v);
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield
        esp_err_t err_i = zmct103c_read_rms(&sens_i, &i_rms);

        // Copia local para cálculos
        float temp_comp = 25.0f;

        // ZONA CRÍTICA: Lectura de temperatura
        if (xSemaphoreTake(sys_mutex, pdMS_TO_TICKS(50))) {
            temp_comp = sys_data.t_gabinete;
            xSemaphoreGive(sys_mutex);
        }

        // Cálculos (fuera del mutex)
        float v_final = 0;
        float i_final = 0;
        bool lecture_valid = (err_v == ESP_OK) && (err_i == ESP_OK);

        if (lecture_valid) {
            float delta_t = temp_comp - 25.0f;
            float factor = 1.0f;
            if (delta_t > 0) factor = 1.0f - (delta_t * 0.0005f);
            
            v_final = r_v.vline_rms * factor;
            i_final = (i_rms > 0.05f) ? i_rms : 0.0f; // Filtro de ruido
        } else {
            ESP_LOGW(TAG, "Error leyendo sensores ADC");
        }

        // ZONA CRÍTICA: Escritura de resultados
        if (xSemaphoreTake(sys_mutex, pdMS_TO_TICKS(50))) {
            if (lecture_valid) {
                sys_data.v_red = v_final;
                sys_data.i_motor = i_final;
                sys_data.sensor_error = false;
            } else {
                sys_data.sensor_error = true;
            }
            xSemaphoreGive(sys_mutex);
        }

        // Actuación LED (Thread safe)
        if (lecture_valid) {
            bool peligro = (v_final < 190 || v_final > 245);
            gpio_set_level(LED_STATUS, peligro);
        } else {
            // Parpadeo por error
            gpio_set_level(LED_STATUS, !gpio_get_level(LED_STATUS));
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    // 1. NVS Robusta
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. Crear Mutex
    sys_mutex = xSemaphoreCreateMutex();
    if (sys_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando Mutex");
        return;
    }

    // 3. GPIO
    gpio_reset_pin(LED_STATUS);
    gpio_set_direction(LED_STATUS, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "SISTEMA V3.0 START");

    // 4. Tareas
    // Térmica: Prio 2 (Duerme mucho)
    xTaskCreate(vTaskTermica, "Task_Temp", 4096, NULL, 2, NULL);
    
    // Eléctrica: Prio 1 (Polling pesado).
    // Fundamental que sea BAJA para no bloquear el sistema.
    xTaskCreate(vTaskElectrica, "Task_Elec", 4096, NULL, 1, NULL);
}
