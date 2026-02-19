/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
#define FDCAN_RETRY_LIMIT 3

// TODO: GENERAL TX CAN IDs
#define CURR_VOLTAGE_SOC_TX_ID 0x6B0
#define DCL_CCL_TEMP_TX_ID 0x6B1
#define HIGH_LOW_CELL_VOLTAGE_TX_ID 0x6B2

// TODO: GENERAL RX CAN IDs
#define INVERTER_VOLTAGE_RX_ID 0xA7

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

// BMS CAN DATAFRAMES + CONTEXT
// TODO
typedef union BMS_DATAFRAME_1 {
	struct __attribute__((packed)) {
		int16_t pack_current;         // Bytes 0–1, signed *0.1 A
		uint16_t pack_voltage;        // Bytes 2–3, unsigned *0.1 V
		uint8_t pack_soc;             // Byte 4, *0.5 %
		uint8_t reserved0;
		uint8_t reserved1;
		uint8_t reserved2;
	} data;
	uint8_t array[8];
} BMS_DATAFRAME_1;

typedef union BMS_DATAFRAME_2 {
	struct __attribute__((packed)) {
		uint16_t pack_dcl;            // Bytes 0–1, unsigned
		uint16_t pack_ccl;             // Byte 2
		int8_t pack_high_temp;        // Byte 4
		int8_t pack_low_temp;         // Byte 5
		uint8_t reserved0;            // Byte 6: blank
		uint8_t reserved1;             // Byte 7
	} data;
	uint8_t array[8];
} BMS_DATAFRAME_2;

typedef union BMS_DATAFRAME_3 {
	struct __attribute__((packed)) {
		uint16_t low_cell_voltage;    // Bytes 0–1, *0.0001 V
		uint16_t high_cell_voltage;   // Bytes 2–3, *0.0001 V
		uint8_t reserved0;            // Byte 4: blank
		uint8_t reserved1;            // Byte 5: blank
		uint8_t reserved2;            // Byte 6: blank
		uint8_t reserved3;             // Byte 7
	} data;
	uint8_t array[8];
} BMS_DATAFRAME_3;

typedef struct {
	FDCAN_TxHeaderTypeDef Curr_Voltage_Soc_TxHeader;
	FDCAN_TxHeaderTypeDef DCL_CCL_Temp_TxHeader;
	FDCAN_TxHeaderTypeDef High_Low_Cell_Voltage_TxHeader;

	BMS_DATAFRAME_1 Curr_Voltage_Soc_DF;
	BMS_DATAFRAME_2 DCL_CCL_Temp_DF;
	BMS_DATAFRAME_3 High_Low_Cell_Voltage_DF;

} FDCAN_BMS_CONTEXT;

extern FDCAN_BMS_CONTEXT FDCAN_BMS;
void configureFDCAN_TxMessage_STD(FDCAN_TxHeaderTypeDef* tx_msg, uint32_t std_id);
void configureFDCAN_TxMessage_EXTD(FDCAN_TxHeaderTypeDef* tx_msg, uint32_t extd_id);

extern FDCAN_RxHeaderTypeDef BMS_RxHeader;
extern uint8_t BMS_RxData[8];
extern uint32_t fdcan_rx_count;
void BMS_CAN_RxHandler();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

