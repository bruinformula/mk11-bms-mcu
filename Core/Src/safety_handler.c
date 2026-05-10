/*
 * safety_handler.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "safety_handler.h"

volatile GPIO_PinState new_shutdown_state;
volatile bool shutdown_debounce_active = false;
volatile uint32_t shutdown_debounce_start;

void process_shutdown_signal() {
	if (HAL_GPIO_ReadPin(SHUTDOWN_POWER_GPIO_Port, SHUTDOWN_POWER_Pin) == GPIO_PIN_SET) {
		if (bms_state == BMS_IDLE || bms_state == BMS_INTERNAL_FAULT || bms_state == BMS_EXTERNAL_FAULT) {
			bms_state = BMS_IDLE;
			determine_operating_state();
		}
	} else {
		// OPEN AIRs!
		HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_RESET);
		// NOTE: Precharge Relay would only ever be closed for precharging.
		// Still, we assert that it is open.
		HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);

		if (bms_state != BMS_INTERNAL_FAULT) {
			bms_state = BMS_EXTERNAL_FAULT;
		}
		reset_operating_state(bms_state);
	}
}

void debounce_shutdown_signal() {
	if (shutdown_debounce_active) {
		if (HAL_GetTick() - shutdown_debounce_start < 500) return;
		if (HAL_GPIO_ReadPin(SHUTDOWN_POWER_GPIO_Port, SHUTDOWN_POWER_Pin) == new_shutdown_state) {
			// Stable change detected!
			process_shutdown_signal();
		}
		shutdown_debounce_active = false;
	}
}

volatile BMS_FaultRegister fault_register;

void BMS_SetFault(uint8_t fault) {
	// CRITICAL REGION
	osMutexWait(FAULT_MUTEXHandle, osWaitForever);
	fault_register.reg |= fault;
    osMutexRelease(FAULT_MUTEXHandle);
}

void BMS_ClearFault(uint8_t fault) {
	// CRITICAL REGION
	osMutexWait(FAULT_MUTEXHandle, osWaitForever);
    fault_register.reg &= ~fault;
    osMutexRelease(FAULT_MUTEXHandle);
}

uint8_t BMS_GetFaultRegister() {
	uint8_t snapshot;
	// CRITICAL REGION
	osMutexWait(FAULT_MUTEXHandle, osWaitForever);
	snapshot = fault_register.reg;
	osMutexRelease(FAULT_MUTEXHandle);
	return snapshot;
}

void BMS_CheckFaultRegister() {
	// CRITICAL REGION
	osMutexWait(FAULT_MUTEXHandle, osWaitForever);
	if (fault_register.reg != 0) {
		HAL_GPIO_WritePin(BMS_FAULT_GPIO_Port, BMS_FAULT_Pin, GPIO_PIN_RESET);
		bms_state = BMS_INTERNAL_FAULT;
	} else {
		// Shutdown Reset needs to be hit for fault to clear on Shutdown Board.
		// Even if the pin is driven high (NO FAULT) on the BMS.
		HAL_GPIO_WritePin(BMS_FAULT_GPIO_Port, BMS_FAULT_Pin, GPIO_PIN_SET);
	}
	osMutexRelease(FAULT_MUTEXHandle);
}
