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
#include "stm32g4xx_hal.h"

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
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define NEG_AIR_GND_Pin GPIO_PIN_0
#define NEG_AIR_GND_GPIO_Port GPIOC
#define POS_AIR_GND_Pin GPIO_PIN_1
#define POS_AIR_GND_GPIO_Port GPIOC
#define PRECHARGE_Pin GPIO_PIN_3
#define PRECHARGE_GPIO_Port GPIOC
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define CS3_Pin GPIO_PIN_4
#define CS3_GPIO_Port GPIOA
#define CHR_DCHG_ENA_Pin GPIO_PIN_6
#define CHR_DCHG_ENA_GPIO_Port GPIOA
#define J17712_PILOT_SWITCH_SIG_Pin GPIO_PIN_0
#define J17712_PILOT_SWITCH_SIG_GPIO_Port GPIOB
#define SHUTDOWN_PWR_Pin GPIO_PIN_10
#define SHUTDOWN_PWR_GPIO_Port GPIOB
#define READY_SIG_Pin GPIO_PIN_11
#define READY_SIG_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_12
#define CS2_GPIO_Port GPIOB
#define BMS_GPIO_OUT_1_Pin GPIO_PIN_6
#define BMS_GPIO_OUT_1_GPIO_Port GPIOC
#define BMS_GPIO_IN_2_Pin GPIO_PIN_7
#define BMS_GPIO_IN_2_GPIO_Port GPIOC
#define BMS_GPIO_OUT_3_Pin GPIO_PIN_9
#define BMS_GPIO_OUT_3_GPIO_Port GPIOC
#define CHARGE_SIG_Pin GPIO_PIN_9
#define CHARGE_SIG_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
