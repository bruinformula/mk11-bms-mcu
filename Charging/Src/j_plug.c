/*
 * j_plug.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "j_plug.h"

J1772_CONTEXT j1772_context;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    	uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t pulse  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    	if (period != 0) {
            j1772_context.control_pilot_duty_cycle = ((float)pulse * 100.0f) / period;
            j1772_context.control_pilot_freq = (float)TIMER_CLOCK / period;

            if (j1772_context.control_pilot_freq < 995 || j1772_context.control_pilot_freq > 1005) {
            	// Frequency should be ~1000 Hz, i.e. 1 Khz as per J1772 Standard
            	j1772_context.control_pilot_state = STATE_CP_ERROR;
            	return;
            }

            if (j1772_context.control_pilot_duty_cycle < 3.0 || j1772_context.control_pilot_duty_cycle > 97.0) {
            	// Duty Cycle too low ==> 0 Amps Advertised
            	// Duty Cycle too high ==> Weird idle voltage signal
            	j1772_context.control_pilot_state = STATE_CP_ERROR;
            	return;
            }

            j1772_context.control_pilot_state = STATE_CP_PWM_PRESENT;

            if (j1772_context.control_pilot_duty_cycle <= 85) {
                j1772_context.control_pilot_advertised_amps = (int)(j1772_context.control_pilot_duty_cycle*0.6);
            } else {
                j1772_context.control_pilot_advertised_amps = (int)((j1772_context.control_pilot_duty_cycle-64)*2.5);
            }
    	}
    }
}

void readControlPilot() {
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}

void stopReadingControlPilot() {
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);
}

void readProximityPilot() {
	j1772_context.proximity_pilot_voltage = (j1772_context.proximity_pilot_adc/4095.0)*3.3;

	// TODO: NEED TO EXAMINE ACTUAL VOLTAGE VALUES AFTER DIVIDER AND SET THRESHOLDS!
	if (fabsf(j1772_context.proximity_pilot_voltage - 4.4) < PP_VOLTAGE_EPSILON) {
		j1772_context.proximity_pilot_state = STATE_PP_NOT_CONNECTED;
	} else if (fabsf(j1772_context.proximity_pilot_voltage - 2.7) < PP_VOLTAGE_EPSILON) {
		j1772_context.proximity_pilot_state = STATE_PP_BUTTON_PRESSED;
	} else if (fabsf(j1772_context.proximity_pilot_voltage - 1.5) < PP_VOLTAGE_EPSILON) {
		j1772_context.proximity_pilot_state= STATE_PP_CONNECTED;
	}
}
