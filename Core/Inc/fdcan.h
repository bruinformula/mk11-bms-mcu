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


// TODO: TX CAN IDs
#define CURR_VOLTAGE_SOC_TX_ID 0x6B0
#define DCL_CCL_TEMP_TX_ID 0x6B1
#define HIGH_LOW_CELL_VOLTAGE_TX_ID 0x6B2
#define PRECHARGE_COMPLETE_TX_ID 0x6B4

// TODO: RX CAN IDs
#define PRECHARGE_REQUEST_RX_ID 0x6B3
#define INVERTER_VOLTAGE_RX_ID 0xA7

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

// BMS CAN DATAFRAMES + CONTEXT
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

typedef union BMS_DATAFRAME_4 {
	struct __attribute__((packed)) {
		uint8_t inverter_precharged;
		uint8_t reserved0;
		uint8_t reserved1;
		uint8_t reserved2;
		uint8_t reserved3;
		uint8_t reserved4;
		uint8_t reserved5;
		uint8_t reserved6;
		uint8_t reserved7;
	} data;
	uint8_t array[8];
} BMS_DATAFRAME_4;

typedef struct {
	FDCAN_TxHeaderTypeDef Curr_Voltage_Soc_TxHeader;
	FDCAN_TxHeaderTypeDef DCL_CCL_Temp_TxHeader;
	FDCAN_TxHeaderTypeDef High_Low_Cell_Voltage_TxHeader;
	FDCAN_TxHeaderTypeDef Precharge_Complete_TxHeader;

	BMS_DATAFRAME_1 Curr_Voltage_Soc_DF;
	BMS_DATAFRAME_2 DCL_CCL_Temp_DF;
	BMS_DATAFRAME_3 High_Low_Cell_Voltage_DF;
	BMS_DATAFRAME_4 Precharge_Complete_DF;

} FDCAN_BMS_CONTEXT;

// CAN CHARGER DATAFRAMES + CONTEXT
typedef union CHARGER_MSG_1806e7f4 {
	struct __attribute__((packed)) {
		uint16_t pack_voltage;        // Bytes 0-1, unsigned *0.1 V
		int16_t pack_ccl;             // Byte 2-3, signed *0.1A
		uint8_t charge_enable;            // Byte 4: charge enable, 0 to charge and 1 to stop charging
		uint8_t reserved0;            // Byte 5: blank
		uint8_t reserved1;            // Byte 6: blank
		uint8_t reserved2;             // Byte 7
	} data;
	uint8_t array[8];
} msg_1806e7f4;

typedef union CHARGER_MSG_1806e5f4 {
	struct __attribute__((packed)) {
		uint16_t high_cell_voltage;   // Bytes 2–3, *0.0001 V
		uint16_t pack_ccl;             // Byte 2-3, signed *0.1A
		uint8_t reserved0;            // Byte 4: blank
		uint8_t reserved1;            // Byte 5: blank
		uint8_t reserved2;            // Byte 6: blank
		uint8_t reserved3;             // Byte 7
	} data;
	uint8_t array[8];
} msg_1806e5f4;

typedef union CHARGER_MSG_1806e9f4 {
	struct __attribute__((packed)) {
		uint16_t high_cell_voltage;   // Bytes 2–3, *0.0001 V
		uint16_t pack_ccl;             // Byte 2-3, signed *0.1A
		uint8_t reserved0;            // Byte 4: blank
		uint8_t reserved1;            // Byte 5: blank
		uint8_t reserved2;            // Byte 6: blank
		uint8_t reserved3;             // Byte 7
	} data;
	uint8_t array[8];
} msg_1806e9f4;

typedef union CHARGER_MSG_18ff50e5 {
	struct __attribute__((packed)) {
		uint8_t reserved0;            // Byte 0: blank
		uint8_t reserved1;            // Byte 1: blank
		uint8_t reserved2;            // Byte 2: blank
		uint8_t reserved3;             // Byte 3: blank
		uint8_t reserved4;            // Byte 4: blank
		uint8_t reserved5;            // Byte 5: blank
		uint8_t reserved6;            // Byte 6: blank
		uint8_t reserved7;             // Byte 7: blank
	} data;
	uint8_t array[8];
} msg_18ff50e5;

typedef struct FDCAN_CHARGER_CONTEXT{
	FDCAN_TxHeaderTypeDef TxHeader_1806E7F4;
	FDCAN_TxHeaderTypeDef TxHeader_1806E5F4;
	FDCAN_TxHeaderTypeDef TxHeader_1806E9F4;
	FDCAN_TxHeaderTypeDef TxHeader_18FF50E5;

	msg_1806e7f4 chgmsg_1806e7f4_DF;
	msg_1806e5f4 chgmsg_1806e5f4_DF;
	msg_1806e9f4 chgmsg_1806e9f4_DF;
	msg_18ff50e5 chgmsg_18ff50e5_DF;
} FDCAN_CHARGER_CONTEXT;

extern FDCAN_BMS_CONTEXT FDCAN_BMS;
extern FDCAN_CHARGER_CONTEXT FDCAN_CHARGER;
void configureCAN_TxMessage(FDCAN_TxHeaderTypeDef* tx_msg, uint32_t std_id);

extern FDCAN_RxHeaderTypeDef BMS_RxHeader;
extern uint8_t BMS_RxData[8];
void BMS_CAN_RxHandler();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

