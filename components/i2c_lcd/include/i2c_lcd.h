#pragma once
#include "esp_err.h"

// Dirección I2C común: 0x27 (o 0x3F en algunos modelos)
#define LCD_ADDR 0x27 
#define LCD_COLS 16
#define LCD_ROWS 2

// Inicializa el bus I2C y el LCD
void lcd_init(void);

// Envía texto
void lcd_send_string(char *str);

// Mueve el cursor (fila 0-1, columna 0-15)
void lcd_set_cursor(int row, int col);

// Limpia la pantalla
void lcd_clear(void);