#pragma once
#include "esp_err.h"
#include <stdbool.h>

// Dirección I2C común: 0x27 (o 0x3F en algunos modelos)
#define LCD_ADDR 0x27 
#define LCD_COLS 16
#define LCD_ROWS 2

// Inicializa el bus I2C y el LCD
void lcd_init(void);

// Envía texto (filtra caracteres no imprimibles)
void lcd_send_string(const char *str);

// Mueve el cursor (fila 0-1, columna 0-15)
void lcd_set_cursor(int row, int col);

// Limpia la pantalla
void lcd_clear(void);

// Regresa cursor a posición inicial
void lcd_home(void);

// Controla el backlight (encendido/apagado)
void lcd_backlight(bool on);

// Imprime texto formateado en posición específica
void lcd_printf(int row, int col, const char *format, ...);