/*
 * safety_handler.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "safety_handler.h"

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
