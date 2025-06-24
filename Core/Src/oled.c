/*
 * oled.c
 *
 *  Created on: Jun 19, 2025
 *      Author: arek2
 */


#include "oled.h"

void OLED_init(void) {
	SSD1306_init();
}


void OLED_print(char *str, uint16_t x, uint16_t y, uint8_t size) {
	GFX_draw_string(x, y, str, WHITE, BLACK, size, size);
	SSD1306_display_repaint();
}

void OLED_clear(void) {
	SSD1306_display_clear();
}
