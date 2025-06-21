#ifndef __LCD_I2C_H__
#define __LCD_I2C_H__

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#define LCD_ADDR         (0x27 << 1) // przesunięty adres I2C
#define LCD_BACKLIGHT    0x08
#define LCD_ENABLE       0x04
#define LCD_READ_WRITE   0x02
#define LCD_REGISTER_SEL 0x01

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_put_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);

#endif
