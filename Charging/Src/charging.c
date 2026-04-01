/*
 * charging.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "charging.h"

static CHARGING_STATE charging_state = CHG_IDLE;

void change_baud_rate() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON CHARGER
	HAL_FDCAN_Stop(&hfdcan1);
	HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.DataPrescaler = 40;
	HAL_FDCAN_Init(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan1);
}

void charging_sequence_startup() {
	change_baud_rate();
	while (proximity_pilot_state != STATE_PP_CONNECTED) {
		// BLOCK
	}

	uint32_t start_time = HAL_GetTick();
	readControlPilot();
	while (HAL_GetTick() - start_time < 3000) {
		// WAIT 3 SECONDS
	}
	stopReadingControlPilot();

	if (advertised_amps > 0 && control_pilot_state == STATE_CP_CONNECTED) {
		HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_SET);
		get_initial_soc();
		charging_sequence();
	}
}

static uint32_t last_tx_time = 0;
static float requested_voltage = 0;
static float requested_amps = 0;
void charging_sequence() {
	charging_state = CHG_ACTIVE;

	while(1) {
		computeAllVoltages(TOTAL_IC, IC);
		computeAllTemps(TOTAL_IC, IC);

		if (chargerFaultDetected()) {
			sendChargerRequest(0, 0, 1);
			charging_state = CHG_ELCON_FAULT;
			break;
		}

		if (highest_cell_temp > MAX_TEMPERATURE_CHARGING_THRESHOLD) {
			sendChargerRequest(0, 0, 1);
			charging_state = CHG_TEMP_FAULT;
			break;
		}

		if (fabsf(current_sensor_val - requested_amps) > CURRENT_SENSOR_EPSILON) {
			sendChargerRequest(0, 0, 1);
			charging_state = CHG_CURRENT_FAULT;
			break;
		}

		if (highest_cell_voltage > MAX_CELL_VOLTAGE_CHARGING_THRESHOLD) {
			sendChargerRequest(0, 0, 1);
			charging_state = CHG_COMPLETE;
			break;
		}

		if (proximity_pilot_state != STATE_PP_CONNECTED) {
			requested_voltage = 0;
			requested_amps = 0;
		} else {
			requested_voltage = CHARGER_VOLTAGE;
			requested_amps = advertised_amps;
		}

		if (HAL_GetTick() - last_tx_time >= 1000) {
			coulomb_count(HAL_GetTick() - last_tx_time);
			sendChargerRequest(requested_voltage, requested_amps, 0);
			last_tx_time = HAL_GetTick();
		}
	}
}
