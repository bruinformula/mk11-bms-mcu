// bms_hv_io.h
#pragma once
#include "main.h"
#include <stdbool.h>
#include <stm32g474xx.h>

static inline void AIRs_OpenAll(void) {
  HAL_GPIO_WritePin(GPIOB, NEG_AIR_GND_Pin, GPIO_PIN_RESET);      // AIR-
  HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_RESET); // PRECH
  HAL_GPIO_WritePin(GPIOC, POS_AIR_GND_Pin, GPIO_PIN_RESET);      // AIR+
}

static inline void AIR_CloseMinus(void) {
  HAL_GPIO_WritePin(GPIOB, NEG_AIR_GND_Pin, GPIO_PIN_SET);
}

static inline void AIR_ClosePlus(void) {
  HAL_GPIO_WritePin(GPIOC, POS_AIR_GND_Pin, GPIO_PIN_SET);
}

static inline void PRECH_Close(void) {
  HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_SET);
}

static inline void PRECH_Open(void) {
  HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_RESET);
}
