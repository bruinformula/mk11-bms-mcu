/*
 * charging.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "charging.h"

CHARGING_STATE charging_state = CHG_IDLE;

void change_baud_rate() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON CHARGER
	HAL_FDCAN_Stop(&hfdcan1);
	HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.DataPrescaler = 40;
	HAL_FDCAN_Init(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan1);
}

// TODO: j1772_context can update due to TIM Callback for computing control pilot.
void charging_loop() {
	readProximityPilot();

	switch (charging_state) {
		case CHG_IDLE:
			HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_RESET);
			sendChargerRequest(0, 0, 1);

			if (j1772_context.proximity_pilot_state == STATE_PP_CONNECTED
					&& j1772_context.control_pilot_state == STATE_CP_PWM_PRESENT) {
				charging_state = CHG_WAITING;
			}

			break;

		case CHG_WAITING:
			if (j1772_context.proximity_pilot_state != STATE_PP_CONNECTED ||
					j1772_context.control_pilot_state == STATE_CP_ERROR) {
				charging_state = CHG_IDLE;
				break;
			}

			if (j1772_context.control_pilot_state == STATE_CP_PWM_PRESENT) {
				HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_SET);
				charging_state = CHG_ACTIVE;
			}

			break;

		case CHG_ACTIVE:
			if (j1772_context.proximity_pilot_state != STATE_PP_CONNECTED ||
					j1772_context.control_pilot_state == STATE_CP_ERROR) {
				charging_state = CHG_IDLE;
				break;
			}

			if (voltage_context.highest_cell_voltage > MAX_CELL_VOLTAGE_CHARGING_THRESHOLD) {
				charging_state = CHG_COMPLETE;
			} else {
				sendChargerRequest(CHARGER_VOLTAGE, j1772_context.control_pilot_advertised_amps, 0);
			}
			break;

		case CHG_ELCON_FAULT: // CHG_ELCON_FAULT issued in fdcan.c.
			// LATCHING, should power cycle & re-connect charger.
			HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_RESET);
			sendChargerRequest(0,0,1);
			break;

		case CHG_COMPLETE:
			HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_RESET);
			sendChargerRequest(0,0,1);
			break;

	}
}
