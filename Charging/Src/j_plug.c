/*
 * j_plug.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "adc.h"
#include "tim.h"
#include "j_plug.h"

STATE_CP control_pilot_state;
STATE_PP proximity_pilot_state;

void readProximityPilot() {
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedPollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t pp_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	float pp_voltage = (pp_raw/4095.0)*3.3;

	// TODO; NEED TO EXAMINE ACTUAL VOLTAGE VALUES AND SET THRESHOLDS
	if (pp_voltage == 3.0) {
		proximity_pilot_state = STATE_PP_NOT_CONNECTED;
	} else if (pp_voltage == 2.0) {
		proximity_pilot_state = STATE_PP_BUTTON_PRESSED;
	} else if (pp_voltage == 1.0) {
		proximity_pilot_state= STATE_PP_CONNECTED;
	}
}

float duty = 0;
float frequency = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    	uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t pulse  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    	if (period != 0) {
            duty = ((float)pulse * 100.0f) / period;
            frequency = (float)TIMER_CLOCK / period;
    	}
    }
}

void readControlPilotCurrent() {
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}
