#include "lcd_i2c.h"

static I2C_HandleTypeDef *lcd_i2c;

void lcd_send_internal(uint8_t data, uint8_t rs) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble  = (data << 4) & 0xF0;
    uint8_t data_arr[4];

    data_arr[0] = high_nibble | rs | LCD_BACKLIGHT | LCD_ENABLE;
    data_arr[1] = high_nibble | rs | LCD_BACKLIGHT;
    data_arr[2] = low_nibble | rs | LCD_BACKLIGHT | LCD_ENABLE;
    data_arr[3] = low_nibble | rs | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_arr, 4, HAL_MAX_DELAY);
}

void lcd_send_cmd(uint8_t cmd) {
    lcd_send_internal(cmd, 0x00);
    HAL_Delay(2);
}

void lcd_send_data(uint8_t data) {
    lcd_send_internal(data, LCD_REGISTER_SEL);
    HAL_Delay(1);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_put_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = 0;
    switch (row) {
        case 0: addr = 0x00 + col; break;
        case 1: addr = 0x40 + col; break;
        case 2: addr = 0x14 + col; break;
        case 3: addr = 0x54 + col; break;
    }
    lcd_send_cmd(0x80 | addr);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

void lcd_init(I2C_HandleTypeDef *hi2c) {
    lcd_i2c = hi2c;
    HAL_Delay(50);

    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20); // Tryb 4-bitowy

    lcd_send_cmd(0x28); // 4-bit, 2-linie, 5x8 font
    lcd_send_cmd(0x08); // Display off
    lcd_send_cmd(0x01); // Clear
    HAL_Delay(2);
    lcd_send_cmd(0x06); // Entry mode
    lcd_send_cmd(0x0C); // Display on, cursor off, blink off
}
