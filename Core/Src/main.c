/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "oled.h"
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
}

int8_t counter = 0;
uint16_t cursor_cnt = 0;

char time_str[32];
char time_hour_str[16], time_minute_str[16];

extern TIM_HandleTypeDef htim1;

enum SelectionMode selection_mode = SELECTION_NONE;
enum SelectionMode clock_selection_mode = SELECTION_NONE;

enum ButtonAction user_button_action = USER_BTN_EDIT_CLOCK;

struct AlarmConfig alarm_config = {0};
struct ClockConfig clock_config = {0};

bool is_update_lcd = false, is_update_oled = true, is_clock_edit = false, is_lcd_cls = false;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
   alarm_config.enabled = false;
   alarm_config.dismissed = false;
   clock_config.hours = 0;
   clock_config.minutes = 0;
   clock_config.seconds = 0;
   clock_config.date = 1;
   clock_config.month = 1;
   clock_config.year = 25;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  OLED_init();
  LCD_init(&hi2c3);

  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(is_update_lcd) {
		  is_update_lcd = false;
		  LCD_update();
	  }

	  if(is_update_oled) {
		  ENC_ALARM_update();
		  OLED_update_time();
	  }

	  ENC_CLOCK_update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void OLED_update_time(void) {

    OLED_print("Ustaw godzin\313", 0, OLED_ROW_1, 1);

    if(selection_mode != SELECTION_NONE) {

    	if(selection_mode == SELECTION_HOUR) {

    		if(cursor_cnt < 40) {
    			sprintf(time_hour_str, "%02d", alarm_config.hours);
    		} else {
    			sprintf(time_hour_str, "%s", "  ");
    		}
    		sprintf(time_minute_str, "%02d", alarm_config.minutes);

    	} else {

    		if(cursor_cnt < 40) {
        		sprintf(time_minute_str, "%02d", alarm_config.minutes);
			} else {
				sprintf(time_minute_str, "%s", "  ");
			}
    		sprintf(time_hour_str, "%02d", alarm_config.hours);
    	}

    } else {
		sprintf(time_hour_str, "%02d", alarm_config.hours);
		sprintf(time_minute_str, "%02d", alarm_config.minutes);
    }

	if(cursor_cnt > 50) cursor_cnt = 0;
	else cursor_cnt++;

	sprintf(time_str, "%s:%s", time_hour_str, time_minute_str);
	OLED_print(time_str, 0, OLED_ROW_5, 2);
}

void ENC_ALARM_update(void) {
	static uint16_t last_cnt = 0;
	int diff = htim1.Instance->CNT - last_cnt;

	if(diff >= 4 || diff <= -4) {
		diff /= 4;

		switch(selection_mode) {
			case SELECTION_HOUR:
				alarm_config.hours += (int8_t)diff;

				if(alarm_config.hours > 23) alarm_config.hours = 0;
				if (alarm_config.hours < 0) alarm_config.hours = 23;
				break;
			case SELECTION_MINUTE:
				alarm_config.minutes += (int8_t)diff;

				if(alarm_config.minutes > 59) alarm_config.minutes = 0;
				if (alarm_config.minutes < 0) alarm_config.minutes = 59;
			default:
				break;
		}

		last_cnt = htim1.Instance->CNT;
	}
}

void ENC_CLOCK_update(void) {
	static uint16_t last_cnt_clock = 0;
	int diff = htim2.Instance->CNT - last_cnt_clock;

	if(diff >= 4 || diff <= -4) {
		diff /= 4;

		printf("Clock: %d \n", diff);

		switch(clock_selection_mode) {
			case SELECTION_HOUR:
				clock_config.hours += (int8_t)diff;

				if(clock_config.hours > 23) clock_config.hours = 0;
				if (clock_config.hours < 0) clock_config.hours = 23;
				break;

			case SELECTION_MINUTE:
				clock_config.minutes += (int8_t)diff;

				if(clock_config.minutes > 59) clock_config.minutes = 0;
				if (clock_config.minutes < 0) clock_config.minutes = 59;
				break;

			case SELECTION_SECONDS:
				clock_config.seconds += (int8_t)diff;

				if(clock_config.seconds > 59) clock_config.seconds = 0;
				if (clock_config.seconds < 0) clock_config.seconds = 59;
				break;

			case SELECTION_DATE:
				clock_config.date += (int8_t)diff;

				if(clock_config.date > 31) clock_config.date = 1;
				if (clock_config.date < 0) clock_config.date = 31;
				break;

			case SELECTION_MONTH:
				clock_config.month += (int8_t)diff;

				if(clock_config.month > 12) clock_config.month = 1;
				if (clock_config.month < 0) clock_config.month = 12;
				break;

			case SELECTION_YEAR:
				clock_config.year += (int8_t)diff;

				if(clock_config.year > 30) clock_config.year = 25;
				if (clock_config.year < 25) clock_config.year = 30;
			default:
				break;
		}

		last_cnt_clock = htim2.Instance->CNT;
	}
}

void LCD_show_clock_screen() {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);  // Musi być po GetTime()

	char buffer[64];
	sprintf(buffer, "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
	LCD_put_cursor(0, 6);
	LCD_send_string(buffer);

	sprintf(buffer, "%02d.%02d.%d", sDate.Date, sDate.Month, 2000 + sDate.Year);
	LCD_put_cursor(2, 5);
	LCD_send_string(buffer);
}

void LCD_show_edit_clock_screen() {
	char buffer[64];

	LCD_put_cursor(0, 0);
	LCD_send_string("Godzina");
	LCD_put_cursor(0, 10);
	LCD_send_string("Data");
	LCD_put_cursor(3, 0);
	LCD_send_string("Zapisz - User Button");

	sprintf(buffer, "%02d:%02d:%02d", clock_config.hours, clock_config.minutes, clock_config.seconds);
	LCD_put_cursor(1, 0);
	LCD_send_string(buffer);

	sprintf(buffer, "%02d.%02d.%d", clock_config.date, clock_config.month, 2000 + clock_config.year);
	LCD_put_cursor(1, 10);
	LCD_send_string(buffer);

	switch(clock_selection_mode) {
		case SELECTION_HOUR:
			LCD_clear_row(2);
			LCD_put_cursor(2, 0);
			LCD_send_string("^^");
			break;

		case SELECTION_MINUTE:
			LCD_clear_row(2);
			LCD_put_cursor(2, 3);
			LCD_send_string("^^");;
			break;

		case SELECTION_SECONDS:
			LCD_clear_row(2);
			LCD_put_cursor(2, 6);
			LCD_send_string("^^");
			break;

		case SELECTION_DATE:
			LCD_clear_row(2);
			LCD_put_cursor(2, 10);
			LCD_send_string("^^");
			break;

		case SELECTION_MONTH:
			LCD_clear_row(2);
			LCD_put_cursor(2, 13);
			LCD_send_string("^^");
			break;

		case SELECTION_YEAR:
			LCD_clear_row(2);
			LCD_put_cursor(2, 18);
			LCD_send_string("^^");

		default:
			break;
	}
}

void LCD_show_alarm_screen() {
	LCD_put_cursor(2, 3);
	LCD_send_string("     ALARM    ");
	HAL_Delay(150);
	LCD_put_cursor(2, 3);
	LCD_send_string("   ! ALARM !  ");
	HAL_Delay(150);
	LCD_put_cursor(2, 3);
	LCD_send_string("  !! ALARM !! ");
	HAL_Delay(150);
	LCD_put_cursor(2, 3);
	LCD_send_string(" !!! ALARM !!!");
	HAL_Delay(150);
}

void LCD_update(void) {
	if(alarm_config.enabled) {
		LCD_show_alarm_screen();
	} else {
		if(alarm_config.dismissed) {
			LCD_clear();

			LCD_put_cursor(2, 2);
			LCD_send_string("ALARM ODRZUCONY!");
			HAL_Delay(800);
			LCD_clear();
		}
		alarm_config.dismissed = false;

		if(is_clock_edit) {
			if(is_lcd_cls) {
				LCD_clear();
				is_lcd_cls = false;
			}
			LCD_show_edit_clock_screen();
		} else {
			LCD_show_clock_screen();
		}
	}
	is_update_lcd = false;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
