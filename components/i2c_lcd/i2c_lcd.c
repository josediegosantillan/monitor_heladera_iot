#include "i2c_lcd.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "LCD";

// --- PINES I2C ---
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
// Bajamos la velocidad a 50kHz para mayor estabilidad eléctrica
#define I2C_MASTER_FREQ_HZ          50000 
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;  // en=1, rs=0
    data_t[1] = data_u | 0x08;  // en=0, rs=0
    data_t[2] = data_l | 0x0C;  // en=1, rs=0
    data_t[3] = data_l | 0x08;  // en=0, rs=0
    // Aumentamos el timeout a 100ms por seguridad
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data_t, 4, pdMS_TO_TICKS(100));
    return err;
}

static esp_err_t lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;  // en=1, rs=1
    data_t[1] = data_u | 0x09;  // en=0, rs=1
    data_t[2] = data_l | 0x0D;  // en=1, rs=1
    data_t[3] = data_l | 0x09;  // en=0, rs=1
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data_t, 4, pdMS_TO_TICKS(100));
    return err;
}

void lcd_init(void)
{
    i2c_master_init();
    // Espera inicial larga (100ms) para que el voltaje del LCD se estabilice
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // --- SECUENCIA DE RESET ---
    // Enviar 0x30 tres veces con pausas seguras (>10ms cada una)
    lcd_send_cmd(0x30); vTaskDelay(pdMS_TO_TICKS(20)); // Esperar 2 ticks
    lcd_send_cmd(0x30); vTaskDelay(pdMS_TO_TICKS(10)); // Esperar 1 tick
    lcd_send_cmd(0x30); vTaskDelay(pdMS_TO_TICKS(10)); // Esperar 1 tick
    
    // Cambiar a modo 4-bits
    lcd_send_cmd(0x20); vTaskDelay(pdMS_TO_TICKS(10)); 

    // --- CONFIGURACION FINAL ---
    lcd_send_cmd(0x28); // Function set: 4-bit, 2 lines, 5x8 dots
    vTaskDelay(pdMS_TO_TICKS(10));
    
    lcd_send_cmd(0x08); // Display off
    vTaskDelay(pdMS_TO_TICKS(10));
    
    lcd_send_cmd(0x01); // Clear display (Este comando tarda más, damos 20ms)
    vTaskDelay(pdMS_TO_TICKS(20));
    
    lcd_send_cmd(0x06); // Entry mode: Increment cursor
    vTaskDelay(pdMS_TO_TICKS(10));
    
    lcd_send_cmd(0x0C); // Display on, Cursor off
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "LCD Inicializado Correctamente (Modo Seguro)");
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}

void lcd_set_cursor(int row, int col)
{
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    addr += col;
    lcd_send_cmd(addr);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(20)); // El clear necesita tiempo fisico
}
