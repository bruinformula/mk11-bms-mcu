/*
 * charging.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "charging.h"

void change_baud_rate() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON
	HAL_FDCAN_Stop(&hfdcan1);
	HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.DataPrescaler = 40;
	HAL_FDCAN_Init(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan1);
}

void charging_sequence_startup() {
	change_baud_rate();

	while (proximity_pilot_state != STATE_PP_CONNECTED) {
		readProximityPilot();
	}

	uint32_t start_time = HAL_GetTick();
	readControlPilotCurrent();
	while (HAL_GetTick() - start_time < 3000) {
		// BLOCK, SAMPLE CONTROL PILOT SIGNAL FOR 3 SECONDS
	}
	stopReadingControlPilotCurrent();

	if (requested_amps > 0 && control_pilot_state == STATE_CP_CONNECTED) {
		HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port,
				J1772_PILOT_SWITCH_Pin,
				GPIO_PIN_SET);
		charging_sequence();
	}
}

void charging_sequence() {
	computeAllTemps(TOTAL_IC, IC);
	computeAllVoltages(TOTAL_IC, IC);

	if (highest_cell_voltage > MAX_CELL_VOLTAGE_CHARGING_THRESHOLD) {
		HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port,
				J1772_PILOT_SWITCH_Pin,
				GPIO_PIN_SET);
		sendChargerRequest(CHARGER_VOLTAGE, requested_amps, 1);
		return;
	}

	if (highest_cell_temp > MAX_TEMPERATURE_CHARGING_THRESHOLD) {
		HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port,
						J1772_PILOT_SWITCH_Pin,
						GPIO_PIN_SET);
		sendChargerRequest(CHARGER_VOLTAGE, requested_amps, 1);
		return;
	}

	readProximityPilot();
	if (proximity_pilot_state != STATE_PP_CONNECTED) {
		// TODO
		// Block?
	}

	sendChargerRequest(CHARGER_VOLTAGE, requested_amps, 0);
	// TODO: Analyze the ELCON Charger Response for faults on their side.

	HAL_Delay(1000);
}
