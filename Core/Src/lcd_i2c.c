#include "lcd_i2c.h"

static I2C_HandleTypeDef *lcd_i2c;

void LCD_send_internal(uint8_t data, uint8_t rs) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble  = (data << 4) & 0xF0;
    uint8_t data_arr[4];

    data_arr[0] = high_nibble | rs | LCD_BACKLIGHT | LCD_ENABLE;
    data_arr[1] = high_nibble | rs | LCD_BACKLIGHT;
    data_arr[2] = low_nibble | rs | LCD_BACKLIGHT | LCD_ENABLE;
    data_arr[3] = low_nibble | rs | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_arr, 4, HAL_MAX_DELAY);
}

void LCD_send_cmd(uint8_t cmd) {
    LCD_send_internal(cmd, 0x00);
    HAL_Delay(2);
}

void LCD_send_data(uint8_t data) {
    LCD_send_internal(data, LCD_REGISTER_SEL);
    HAL_Delay(1);
}

void LCD_clear(void) {
    LCD_send_cmd(0x01);
    HAL_Delay(2);
}

void LCD_put_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = 0;
    switch (row) {
        case 0: addr = 0x00 + col; break;
        case 1: addr = 0x40 + col; break;
        case 2: addr = 0x14 + col; break;
        case 3: addr = 0x54 + col; break;
    }
    LCD_send_cmd(0x80 | addr);
}

void LCD_send_string(char *str) {
    while (*str) {
        LCD_send_data((uint8_t)(*str));
        str++;
    }
}

void LCD_clear_row(uint8_t row) {
	if(row > LCD_ROWS) {
		row = LCD_ROWS;
	}

	for(uint8_t i = 0; i < LCD_COLS; i++) {
		LCD_put_cursor(row, i);
		LCD_send_string(" ");
	}
}

void LCD_init(I2C_HandleTypeDef *hi2c) {
    lcd_i2c = hi2c;
    HAL_Delay(50);

    LCD_send_cmd(0x30);
    HAL_Delay(5);
    LCD_send_cmd(0x30);
    HAL_Delay(1);
    LCD_send_cmd(0x30);
    HAL_Delay(10);
    LCD_send_cmd(0x20); // Tryb 4-bitowy

    LCD_send_cmd(0x28); // 4-bit, 2-linie, 5x8 font
    LCD_send_cmd(0x08); // Display off
    LCD_send_cmd(0x01); // Clear
    HAL_Delay(2);
    LCD_send_cmd(0x06); // Entry mode
    LCD_send_cmd(0x0C); // Display on, cursor off, blink off
}
