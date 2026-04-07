/*
 * bms_state.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "bms_state.h"

volatile BMS_STATE bms_state = BMS_IDLE;

void determineStartupMode() {
	if (HAL_GPIO_ReadPin(CHARGE_SIGNAL_GPIO_Port, CHARGE_SIGNAL_Pin) == GPIO_PIN_SET) {
		// TODO
		if (HAL_GPIO_ReadPin(BALANCING_EN_GPIO_Port, BALANCING_EN_Pin) == GPIO_PIN_SET) {
			bms_state = BMS_BALANCING;
			fastBalancingLoop(TOTAL_IC, IC);
		} else {
			bms_state = BMS_CHARGING;
			charging_sequence_startup();
			charging_sequence();
		}
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		bms_state = BMS_PRECHARGING;

		while(bms_state == BMS_PRECHARGING) {
			computeAllVoltages(TOTAL_IC, IC);
			HAL_Delay(500);
		}

		if (bms_state == BMS_DRIVE) {
			MX_FREERTOS_Init();
			osKernelStart();
		}
	}
}

void enterDriveMode() {
	// Configure CAN Datalogging Messages
	configureCCL_DCL_TxMsg();
	configureTemp_TxMsg();
	configureVoltage_TxMsg();
	bms_state = BMS_DRIVE;
}
