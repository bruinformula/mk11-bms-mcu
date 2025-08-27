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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define SDC_IN_Pin GPIO_PIN_1
#define SDC_IN_GPIO_Port GPIOC
#define Charge_Enable_Pin GPIO_PIN_2
#define Charge_Enable_GPIO_Port GPIOC
#define Discharge_Enable_Pin GPIO_PIN_3
#define Discharge_Enable_GPIO_Port GPIOC
#define Current_Sensor_Low_Pin GPIO_PIN_1
#define Current_Sensor_Low_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define Current_Sensor_High_Pin GPIO_PIN_4
#define Current_Sensor_High_GPIO_Port GPIOA
#define POS_AIR_GND_Pin GPIO_PIN_4
#define POS_AIR_GND_GPIO_Port GPIOC
#define Charge_Power_Pin GPIO_PIN_5
#define Charge_Power_GPIO_Port GPIOC
#define Temp_Fault_Pin GPIO_PIN_1
#define Temp_Fault_GPIO_Port GPIOB
#define Precharge_Enable_Pin GPIO_PIN_2
#define Precharge_Enable_GPIO_Port GPIOB
#define W1_Pin GPIO_PIN_10
#define W1_GPIO_Port GPIOB
#define NEG_AIR_GND_Pin GPIO_PIN_11
#define NEG_AIR_GND_GPIO_Port GPIOB
#define Cell_Fault_Pin GPIO_PIN_15
#define Cell_Fault_GPIO_Port GPIOB
#define Ready_Power_Pin GPIO_PIN_6
#define Ready_Power_GPIO_Port GPIOC
#define CSB_2_Pin GPIO_PIN_7
#define CSB_2_GPIO_Port GPIOC
#define Always_On_Power_Pin GPIO_PIN_8
#define Always_On_Power_GPIO_Port GPIOC
#define M2_Pin GPIO_PIN_10
#define M2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define I2_Pin GPIO_PIN_3
#define I2_GPIO_Port GPIOB
#define M1_Pin GPIO_PIN_4
#define M1_GPIO_Port GPIOB
#define W2_Pin GPIO_PIN_5
#define W2_GPIO_Port GPIOB
#define CSB1_Pin GPIO_PIN_6
#define CSB1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
