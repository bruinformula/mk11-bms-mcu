/*
 * datalogging.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "datalogging.h"

static TEMP_DF temp_df;
static VOLTAGE_DF voltage_df;

static FDCAN_TxHeaderTypeDef Temp_TxHeader;
static FDCAN_TxHeaderTypeDef Voltage_TxHeader;

void configureTemp_TxMsg() {
	configureFDCAN_TxMessage_STD(&Temp_TxHeader, TEMP_TX_ID);
}

void configureVoltage_TxMsg() {
	configureFDCAN_TxMessage_STD(&Voltage_TxHeader, VOLTAGE_TX_ID);
}

void sendTemp() {
	temp_df.data.avg_temp = (avg_cell_temp*100);
	temp_df.data.highest_temp = (highest_cell_temp*100);
	temp_df.data.lowest_temp = (lowest_cell_temp*100);
	osMutexWait(CAN_MutexHandle, osWaitForever);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Temp_TxHeader, temp_df.array);
	osMutexRelease(CAN_MutexHandle);
}

void sendVoltage() {
	voltage_df.data.avg_cell_voltage = (avg_cell_voltage*100);
	voltage_df.data.highest_cell_voltage = (highest_cell_voltage*100);
	voltage_df.data.lowest_cell_voltage = (lowest_cell_voltage*100);
	voltage_df.data.bms_pack_voltage = (bms_pack_voltage*100);
	osMutexWait(CAN_MutexHandle, osWaitForever);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Voltage_TxHeader, voltage_df.array);
	osMutexRelease(CAN_MutexHandle);
}
