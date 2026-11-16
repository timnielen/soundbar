/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOF
#define BT_EN_Pin GPIO_PIN_0
#define BT_EN_GPIO_Port GPIOA
#define PVDD_SENSE_Pin GPIO_PIN_1
#define PVDD_SENSE_GPIO_Port GPIOA
#define BT_PRESENT_Pin GPIO_PIN_6
#define BT_PRESENT_GPIO_Port GPIOA
#define PDN__Pin GPIO_PIN_0
#define PDN__GPIO_Port GPIOB
#define SW_LEFT_Pin GPIO_PIN_12
#define SW_LEFT_GPIO_Port GPIOB
#define SW_LEFT_EXTI_IRQn EXTI4_15_IRQn
#define ADC_MASTER_Pin GPIO_PIN_13
#define ADC_MASTER_GPIO_Port GPIOB
#define BTN4_Pin GPIO_PIN_15
#define BTN4_GPIO_Port GPIOA
#define BTN4_EXTI_IRQn EXTI4_15_IRQn
#define BTN3_Pin GPIO_PIN_0
#define BTN3_GPIO_Port GPIOD
#define BTN3_EXTI_IRQn EXTI0_1_IRQn
#define BTN2_Pin GPIO_PIN_1
#define BTN2_GPIO_Port GPIOD
#define BTN2_EXTI_IRQn EXTI0_1_IRQn
#define BTN1_Pin GPIO_PIN_2
#define BTN1_GPIO_Port GPIOD
#define BTN1_EXTI_IRQn EXTI2_3_IRQn

/* USER CODE BEGIN Private defines */

#define LEFT_TAS_ADDR 0x5E
#define RIGHT_TAS_ADDR 0x58

HAL_StatusTypeDef tas_deep_sleep(uint8_t addr);
HAL_StatusTypeDef tas_play(uint8_t addr);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
