/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lcd.h"
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define CHAR_W 		11
#define CHAR_H 		18
#define MIN_TIME	10
#define MAX_TIME	180
#define TIME_STEP	10
#define START_TIME	30
#define STOPPED		0
#define RUNNING		1
#define DEBOUNCE_TIME 10
#define TIM14_COUNTS 10
#define TIM17_COUNTS 1000


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void TIM17_IRQ_Callback(void);
void TIM14_IRQ_Callback(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPEED_MINUS_Pin GPIO_PIN_15
#define SPEED_MINUS_GPIO_Port GPIOC
#define SPEED_PLUS_Pin GPIO_PIN_0
#define SPEED_PLUS_GPIO_Port GPIOA
#define LCDCS_Pin GPIO_PIN_3
#define LCDCS_GPIO_Port GPIOA
#define ON_Pin GPIO_PIN_4
#define ON_GPIO_Port GPIOA
#define BACKLIGHT_PWM_Pin GPIO_PIN_6
#define BACKLIGHT_PWM_GPIO_Port GPIOA
#define LCDDC_Pin GPIO_PIN_11
#define LCDDC_GPIO_Port GPIOA
#define LCDRESET_Pin GPIO_PIN_12
#define LCDRESET_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
