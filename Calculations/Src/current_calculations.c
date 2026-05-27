/*
 * current_calculations.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#include "current_calculations.h"

volatile CURRENT_CONTEXT current_context;
static bool using_high_range = false;

void calculateCurrent() {
	// CRITICAL REGION
	taskENTER_CRITICAL();
	current_context.current_sensor_low_voltage = (current_context.current_sensor_low_adc/4095.0)*3.3;
	current_context.current_sensor_high_voltage = (current_context.current_sensor_high_adc/4095.0)*3.3;
	current_context.current_sensor_low = ((float) current_context.current_sensor_low_adc) * 0.01872f - 37.364f;
	current_context.current_sensor_high = ((float) current_context.current_sensor_high_adc) * 0.2174f - 433.79f;
	taskEXIT_CRITICAL();

	if (!using_high_range && fabsf(current_context.current_sensor_low) > 30.0f) {
		using_high_range = true;
	} else if (using_high_range && fabsf(current_context.current_sensor_low) < 25.0f) {
		using_high_range = false;
	}

	current_context.current_sensor_val = using_high_range ?
			current_context.current_sensor_high :
			current_context.current_sensor_low;

	// FAULT HANDLING
	uint8_t faults_set = 0;
	uint8_t faults_clear = 0;
	if (fabsf(current_context.current_sensor_val) > OVERCURRENT_THRESHOLD) {
		faults_set |= FAULT_OVERCURRENT;
	} else {
		faults_clear |= FAULT_OVERCURRENT;
	}

	if (faults_set) {
		BMS_SetFault(faults_set);
	}
	if (faults_clear) {
		BMS_ClearFault(faults_clear);
	}
}
