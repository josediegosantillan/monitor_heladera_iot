#include "i2c_lcd.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "LCD";

// --- PINES I2C ---
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
// Velocidad baja para máxima estabilidad con cables largos
#define I2C_MASTER_FREQ_HZ          50000 
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// Bits de control del PCF8574
#define LCD_BACKLIGHT   0x08  // Bit 3: Backlight on
#define LCD_ENABLE      0x04  // Bit 2: Enable
#define LCD_RW          0x02  // Bit 1: Read/Write (0=Write)
#define LCD_RS          0x01  // Bit 0: Register Select (0=Cmd, 1=Data)

// Reintentos para comunicación I2C
#define I2C_MAX_RETRIES 3

static bool i2c_initialized = false;

static esp_err_t i2c_master_init(void)
{
    if (i2c_initialized) {
        return ESP_OK;
    }
    
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {
        i2c_initialized = true;
        ESP_LOGI(TAG, "I2C inicializado correctamente");
    } else if (err == ESP_ERR_INVALID_STATE) {
        // Driver ya instalado
        i2c_initialized = true;
        err = ESP_OK;
    } else {
        ESP_LOGE(TAG, "Error instalando driver I2C: %s", esp_err_to_name(err));
    }
    
    return err;
}

// Envía un byte al PCF8574 con reintentos
static esp_err_t lcd_write_byte(uint8_t data)
{
    esp_err_t err;
    for (int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
        err = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, pdMS_TO_TICKS(50));
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ets_delay_us(100);
    }
    ESP_LOGW(TAG, "Error I2C después de %d reintentos: %s", I2C_MAX_RETRIES, esp_err_to_name(err));
    return err;
}

// Genera pulso de Enable con timing preciso
static esp_err_t lcd_pulse_enable(uint8_t data)
{
    esp_err_t err;
    
    // Enable HIGH
    err = lcd_write_byte(data | LCD_ENABLE);
    if (err != ESP_OK) return err;
    ets_delay_us(1);  // Enable pulse width >= 450ns
    
    // Enable LOW  
    err = lcd_write_byte(data & ~LCD_ENABLE);
    if (err != ESP_OK) return err;
    ets_delay_us(50); // Tiempo para que el LCD procese el comando
    
    return ESP_OK;
}

// Envía 4 bits al LCD (nibble)
static esp_err_t lcd_send_nibble(uint8_t nibble, bool is_data)
{
    uint8_t data = (nibble & 0xF0) | LCD_BACKLIGHT;
    if (is_data) {
        data |= LCD_RS;  // RS=1 para datos
    }
    // RW siempre 0 (escritura)
    
    return lcd_pulse_enable(data);
}

static esp_err_t lcd_send_cmd(char cmd)
{
    esp_err_t err;
    
    // Nibble alto primero
    err = lcd_send_nibble(cmd & 0xF0, false);
    if (err != ESP_OK) return err;
    
    // Nibble bajo
    err = lcd_send_nibble((cmd << 4) & 0xF0, false);
    if (err != ESP_OK) return err;
    
    // Tiempo extra para comandos lentos (Clear, Home)
    if (cmd == 0x01 || cmd == 0x02) {
        ets_delay_us(2000);  // Clear y Home necesitan ~1.52ms
    } else {
        ets_delay_us(50);    // Otros comandos ~37us
    }
    
    return ESP_OK;
}

static esp_err_t lcd_send_data(char data)
{
    esp_err_t err;
    
    // Nibble alto primero
    err = lcd_send_nibble(data & 0xF0, true);
    if (err != ESP_OK) return err;
    
    // Nibble bajo
    err = lcd_send_nibble((data << 4) & 0xF0, true);
    if (err != ESP_OK) return err;
    
    ets_delay_us(50);  // Tiempo para escribir en RAM
    
    return ESP_OK;
}

void lcd_init(void)
{
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo inicialización I2C");
        return;
    }
    
    // Espera inicial larga (>40ms después de Vcc=2.7V) para estabilización
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // === SECUENCIA DE INICIALIZACIÓN SEGÚN DATASHEET HD44780 ===
    
    // Paso 1: Esperar >15ms después de Vcc=4.5V
    // (ya esperamos 100ms arriba)
    
    // Paso 2: Enviar 0x30 (Function Set 8-bit) - primera vez
    // NOTA: En modo 4-bit, enviamos solo el nibble alto
    lcd_write_byte(0x30 | LCD_BACKLIGHT);
    lcd_pulse_enable(0x30 | LCD_BACKLIGHT);
    vTaskDelay(pdMS_TO_TICKS(5));  // Esperar >4.1ms
    
    // Paso 3: Enviar 0x30 - segunda vez
    lcd_write_byte(0x30 | LCD_BACKLIGHT);
    lcd_pulse_enable(0x30 | LCD_BACKLIGHT);
    ets_delay_us(150);  // Esperar >100us
    
    // Paso 4: Enviar 0x30 - tercera vez
    lcd_write_byte(0x30 | LCD_BACKLIGHT);
    lcd_pulse_enable(0x30 | LCD_BACKLIGHT);
    ets_delay_us(150);
    
    // Paso 5: Cambiar a modo 4-bit (enviar 0x20)
    lcd_write_byte(0x20 | LCD_BACKLIGHT);
    lcd_pulse_enable(0x20 | LCD_BACKLIGHT);
    ets_delay_us(150);
    
    // === AHORA ESTAMOS EN MODO 4-BIT ===
    
    // Function Set: 4-bit, 2 líneas, 5x8 puntos
    lcd_send_cmd(0x28);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Display OFF
    lcd_send_cmd(0x08);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Clear Display (tarda ~1.52ms)
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Entry Mode: Incrementar cursor, sin shift
    lcd_send_cmd(0x06);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Display ON, Cursor OFF, Blink OFF
    lcd_send_cmd(0x0C);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    ESP_LOGI(TAG, "LCD inicializado correctamente (Secuencia HD44780)");
}

void lcd_send_string(const char *str)
{
    if (str == NULL) return;
    
    while (*str) {
        // Filtrar caracteres no imprimibles (solo ASCII 32-126)
        char c = *str++;
        if (c >= 32 && c <= 126) {
            lcd_send_data(c);
        } else if (c == '\n') {
            // Salto de línea: mover a línea 2
            lcd_set_cursor(1, 0);
        }
        // Pequeño delay entre caracteres para estabilidad
        ets_delay_us(50);
    }
}

void lcd_set_cursor(int row, int col)
{
    // Validar límites
    if (row < 0) row = 0;
    if (row > 1) row = 1;
    if (col < 0) col = 0;
    if (col > 15) col = 15;
    
    // Direcciones DDRAM: Fila 0 = 0x00, Fila 1 = 0x40
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_send_cmd(0x80 | addr);  // Set DDRAM Address
    ets_delay_us(50);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5)); // Clear necesita ~1.52ms mínimo
}

void lcd_home(void)
{
    lcd_send_cmd(0x02);
    vTaskDelay(pdMS_TO_TICKS(5)); // Home necesita ~1.52ms mínimo
}

void lcd_backlight(bool on)
{
    uint8_t data = on ? LCD_BACKLIGHT : 0x00;
    lcd_write_byte(data);
}

// Imprime un valor formateado de forma segura
void lcd_printf(int row, int col, const char *format, ...)
{
    char buffer[LCD_COLS + 1];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    lcd_set_cursor(row, col);
    lcd_send_string(buffer);
}
