/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ENC_ALARM_update(void);
void ENC_CLOCK_update(void);
void OLED_update_time(void);
void LCD_update(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define USER_BTN_EXTI_IRQn EXTI0_IRQn
#define ENC2_CLK_Pin GPIO_PIN_1
#define ENC2_CLK_GPIO_Port GPIOA
#define ENC2_DATA_Pin GPIO_PIN_5
#define ENC2_DATA_GPIO_Port GPIOA
#define BUZZER_TIM3_PWM_Pin GPIO_PIN_6
#define BUZZER_TIM3_PWM_GPIO_Port GPIOA
#define ENC1_CLK_Pin GPIO_PIN_9
#define ENC1_CLK_GPIO_Port GPIOE
#define ENC1_DATA_Pin GPIO_PIN_11
#define ENC1_DATA_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
#define ENC_BTN_Pin GPIO_PIN_1
#define ENC_BTN_GPIO_Port GPIOD
#define ENC_BTN_EXTI_IRQn EXTI1_IRQn
#define ENC2_BTN_Pin GPIO_PIN_2
#define ENC2_BTN_GPIO_Port GPIOD
#define ENC2_BTN_EXTI_IRQn EXTI2_IRQn
#define OLED_I2C_SCL_Pin GPIO_PIN_6
#define OLED_I2C_SCL_GPIO_Port GPIOB
#define OLED_I2C_SDA_Pin GPIO_PIN_7
#define OLED_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
enum SelectionMode {
	SELECTION_HOUR,
	SELECTION_MINUTE,
	SELECTION_SECONDS,
	SELECTION_NONE,
	SELECTION_DATE,
	SELECTION_MONTH,
	SELECTION_YEAR
};

enum ButtonAction {
	USER_BTN_EDIT_CLOCK,
	USER_BTN_SAVE_CLOCK,
	USER_BTN_EDIT_ALARM,
	USER_BTN_SAVE_ALARM,
	USER_BTN_DISMISS_ALARM
};

struct AlarmConfig {
	bool enabled;
	bool dismissed;
	uint8_t hours;
	uint8_t minutes;
};

struct ClockConfig {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t date;
	uint8_t month;
	uint16_t year;
};

void RTC_update_clock(struct ClockConfig *config);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
