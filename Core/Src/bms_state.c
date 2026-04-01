/*
 * bms_state.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "bms_state.h"

BMS_STATE bms_state = BMS_IDLE;

void determineStartupMode() {
	if (HAL_GPIO_ReadPin(CHARGE_SIGNAL_GPIO_Port, CHARGE_SIGNAL_Pin) == GPIO_PIN_SET) {
		if (HAL_GPIO_ReadPin(BALANCING_EN_GPIO_Port, BALANCING_EN_Pin) == GPIO_PIN_SET) {
			bms_state = BMS_BALANCING;
			balancingLoop(TOTAL_IC, IC);
		} else {
			bms_state = BMS_CHARGING;
			charging_sequence_startup();
			charging_sequence();
		}
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		bms_state = BMS_PRECHARGING;
	}
}

void enterDriveMode() {
	configureCCL_DCL_TxMsg();
	configureTemp_TxMsg();
	configureVoltage_TxMsg();
	bms_state = BMS_DRIVE;
	osKernelStart();
}

void spi_lock() {
	if (bms_state == BMS_DRIVE) {
		osMutexWait(SPI_MUTEXHandle, osWaitForever);
	}
}

void spi_unlock() {
	if (bms_state == BMS_DRIVE) {
		osMutexRelease(SPI_MUTEXHandle);
	}
}
