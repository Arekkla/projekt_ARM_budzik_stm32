/*
 * oled.h
 *
 *  Created on: Jun 19, 2025
 *      Author: arek2
 */

#include "SSD1306.h"

#ifndef INC_OLED_H_
#define INC_OLED_H_

#endif /* INC_OLED_H_ */

#define OLED_ROW_1 56
#define OLED_ROW_2 48
#define OLED_ROW_3 40
#define OLED_ROW_4 32
#define OLED_ROW_5 24
#define OLED_ROW_6 16
#define OLED_ROW_7 8
#define OLED_ROW_8 0


void OLED_init(void);
void OLED_print(char *str, uint16_t x, uint16_t y, uint8_t size);
void OLED_clear(void);
