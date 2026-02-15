/*
 * elcon.c
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */

#include "elcon_charger.h"

FDCAN_CHARGER_CONTEXT FDCAN_CHARGER;

void configureChargeTxMsg() {
	configureFDCAN_TxMessage_EXTD(&FDCAN_CHARGER.TxHeader_1806E5F4, ELCON_CHARGER_TX_ID);
}

void sendChargerRequest(float max_charging_voltage,
		float max_charging_current,
		bool stop_charging) {

	uint16_t voltage_raw = (uint16_t)(max_charging_voltage*10);
	uint16_t current_raw = (uint16_t)(max_charging_current*10);

	FDCAN_CHARGER.chgmsg_1806E5F4_DF.data.max_charging_voltage_msb = (voltage_raw >> 8) & 0xFF;
	FDCAN_CHARGER.chgmsg_1806E5F4_DF.data.max_charging_voltage_lsb = voltage_raw  & 0xFF;
	FDCAN_CHARGER.chgmsg_1806E5F4_DF.data.max_charging_current_msb = (current_raw >> 8) & 0xFF;
	FDCAN_CHARGER.chgmsg_1806E5F4_DF.data.max_charging_current_lsb = current_raw & 0xFF;
	FDCAN_CHARGER.chgmsg_1806E5F4_DF.data.charging_control = stop_charging;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
			&FDCAN_CHARGER.TxHeader_1806E5F4,
			FDCAN_CHARGER.chgmsg_1806E5F4_DF.array);
}

void parseChargerBroadcast() {
	FDCAN_CHARGER.chgmsg_18FF50E5_DF.data.charger_output_voltage = ((BMS_RxData[0] << 8) | BMS_RxData[1]);
	FDCAN_CHARGER.chgmsg_18FF50E5_DF.data.charger_output_current = ((BMS_RxData[2] << 8) | BMS_RxData[3]);
	FDCAN_CHARGER.chgmsg_18FF50E5_DF.data.status_flags.raw = BMS_RxData[4];
}
