/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define vRangeA_SW_Pin LL_GPIO_PIN_14
#define vRangeA_SW_GPIO_Port GPIOC
#define vRangeB_SW_Pin LL_GPIO_PIN_15
#define vRangeB_SW_GPIO_Port GPIOC
#define NC_Pin LL_GPIO_PIN_3
#define NC_GPIO_Port GPIOA
#define LEFT_Button_Pin LL_GPIO_PIN_6
#define LEFT_Button_GPIO_Port GPIOA
#define LEFT_Button_EXTI_IRQn EXTI9_5_IRQn
#define RIGHT_Button_Pin LL_GPIO_PIN_7
#define RIGHT_Button_GPIO_Port GPIOA
#define RIGHT_Button_EXTI_IRQn EXTI9_5_IRQn
#define HOLD_Button_Pin LL_GPIO_PIN_11
#define HOLD_Button_GPIO_Port GPIOB
#define HOLD_Button_EXTI_IRQn EXTI15_10_IRQn
#define SELECT_Button_Pin LL_GPIO_PIN_12
#define SELECT_Button_GPIO_Port GPIOB
#define SELECT_Button_EXTI_IRQn EXTI15_10_IRQn
#define DOWN_Button_Pin LL_GPIO_PIN_9
#define DOWN_Button_GPIO_Port GPIOA
#define DOWN_Button_EXTI_IRQn EXTI9_5_IRQn
#define UP_Button_Pin LL_GPIO_PIN_10
#define UP_Button_GPIO_Port GPIOA
#define UP_Button_EXTI_IRQn EXTI15_10_IRQn
#define DC_ST7789_Pin LL_GPIO_PIN_11
#define DC_ST7789_GPIO_Port GPIOA
#define RST_ST7789_Pin LL_GPIO_PIN_12
#define RST_ST7789_GPIO_Port GPIOA
#define PWM_1kHz_Pin LL_GPIO_PIN_9
#define PWM_1kHz_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
