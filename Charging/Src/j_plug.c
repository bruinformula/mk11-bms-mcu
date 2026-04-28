/*
 * j_plug.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "j_plug.h"

volatile STATE_CP control_pilot_state;
volatile STATE_PP proximity_pilot_state;
volatile J1772_CONTEXT j1772_context;

int debug_cb = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	debug_cb++;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    	uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t pulse  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    	j1772_context.control_pilot_period = period;
    	j1772_context.control_pilot_pulse = pulse;

    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    	xTaskNotifyFromISR(chargingTaskHandle, EVT_CP_UPDATE, eSetBits, &xHigherPriorityTaskWoken);
    	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void startPWM_Capture() {
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}

void stopPWM_Capture() {
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);
}

STATE_PP readProximityPilot(uint16_t adc) {
	float pp_voltage = (adc/4095.0f)*3.3f;
	if (fabsf(pp_voltage - 2.9f) < PP_VOLTAGE_EPSILON) {
		return STATE_PP_NOT_CONNECTED;
	} else if (fabsf(pp_voltage - 1.8f) < PP_VOLTAGE_EPSILON) {
		return STATE_PP_BUTTON_PRESSED;
	} else if (fabsf(pp_voltage - 1.0f) < PP_VOLTAGE_EPSILON) {
		return STATE_PP_CONNECTED;
	}
	return STATE_PP_UNKNOWN;
}

int advertised_amps;
STATE_CP readControlPilot(uint32_t period, uint32_t pulse) {
	if (period != 0) {
		float duty_cycle = ((float)pulse * 100.0f) / period;
		float freq = (float)TIMER_CLOCK / period;

		if (freq < 995.0f || freq > 1005.0f) {
			return STATE_CP_ERROR;
		}

		if (duty_cycle < 3.0f || duty_cycle > 97.0f) {
			// Duty Cycle too low ==> 0 Amps Advertised
		    // Duty Cycle too high ==> Weird idle voltage signal
		    return STATE_CP_ERROR;
		}

		if (duty_cycle <= 85.0f) {
			advertised_amps = (int)(duty_cycle*0.6f);
		} else {
			advertised_amps = (int)((duty_cycle-64.0f)*2.5f);
		}

		return STATE_CP_PWM_PRESENT;
	}

	return STATE_CP_ERROR;
}
