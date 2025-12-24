#include "ds18b20.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==========================================================
// CORRECCION CRITICA: Definimos el Spinlock como variable estática
// para que persista en memoria y no sea un objeto temporal.
// ==========================================================
static portMUX_TYPE ds18b20_spinlock = portMUX_INITIALIZER_UNLOCKED;

void ds18b20_init(ds18b20_t *sensor, gpio_num_t pin) {
    sensor->pin = pin;
    gpio_reset_pin(pin);
    // Configuración Open Drain: El sensor necesita tirar a masa.
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD); 
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
}

// Funciones auxiliares (sin bloqueo interno, el bloqueo lo hace la función principal)
static bool _reset(gpio_num_t pin) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(480);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(70);
    int presence = gpio_get_level(pin);
    esp_rom_delay_us(410);
    return (presence == 0);
}

static void _write_bit(gpio_num_t pin, int bit) {
    gpio_set_level(pin, 0);
    if (bit) {
        esp_rom_delay_us(6);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(10);
    }
}

static int _read_bit(gpio_num_t pin) {
    int bit = 0;
    gpio_set_level(pin, 0);
    esp_rom_delay_us(6);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(9);
    bit = gpio_get_level(pin);
    esp_rom_delay_us(55);
    return bit;
}

static void _write_byte(gpio_num_t pin, uint8_t data) {
    for (int i = 0; i < 8; i++) _write_bit(pin, (data >> i) & 1);
}

static uint8_t _read_byte(gpio_num_t pin) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) data |= (_read_bit(pin) << i);
    return data;
}

float ds18b20_read_temp(ds18b20_t *sensor) {
    // 1. INICIO DE TRANSACCIÓN (Reset + Comando)
    // Usamos portENTER_CRITICAL_SAFE para deshabilitar interrupciones en ambos cores
    portENTER_CRITICAL_SAFE(&ds18b20_spinlock); 
    
    if (!_reset(sensor->pin)) {
        portEXIT_CRITICAL_SAFE(&ds18b20_spinlock); // Salida segura si falla
        return SENSOR_TEMP_ERROR;
    }
    _write_byte(sensor->pin, 0xCC); // Skip ROM
    _write_byte(sensor->pin, 0x44); // Convert T
    
    portEXIT_CRITICAL_SAFE(&ds18b20_spinlock);

    // 2. ESPERA (No bloqueante para el CPU, sí para la tarea)
    // El sensor tarda hasta 750ms en convertir 12 bits.
    vTaskDelay(pdMS_TO_TICKS(750)); 

    // 3. LECTURA DE DATOS
    portENTER_CRITICAL_SAFE(&ds18b20_spinlock);
    
    if (!_reset(sensor->pin)) {
        portEXIT_CRITICAL_SAFE(&ds18b20_spinlock);
        return SENSOR_TEMP_ERROR;
    }
    _write_byte(sensor->pin, 0xCC);
    _write_byte(sensor->pin, 0xBE); // Read Scratchpad
    
    uint8_t low = _read_byte(sensor->pin);
    uint8_t high = _read_byte(sensor->pin);
    
    portEXIT_CRITICAL_SAFE(&ds18b20_spinlock);

    // Conversión matemática
    int16_t raw = (high << 8) | low;
    
    // El DS18B20 a veces devuelve 85°C (valor de power-on) si leemos muy rápido
    // o si falló la alimentación. Lo dejamos pasar, pero tenelo en cuenta.
    return (float)raw / 16.0f;
}