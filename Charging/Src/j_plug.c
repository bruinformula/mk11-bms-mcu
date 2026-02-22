/*
 * j_plug.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "adc.h"
#include "tim.h"
#include "j_plug.h"

volatile STATE_CP control_pilot_state = STATE_CP_IDLE;
volatile STATE_PP proximity_pilot_state = STATE_PP_IDLE;

volatile float proximity_pilot_voltage;
void readProximityPilot() {
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedPollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t pp_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	proximity_pilot_voltage = (pp_raw/4095.0)*3.3;

	// TODO; NEED TO EXAMINE ACTUAL VOLTAGE VALUES AFTER DIVIDER AND SET THRESHOLDS...
	// SHOULD ALSO BE WITHIN SOME EPSILON OF THESE STATES
	if (proximity_pilot_voltage == 3.0) {
		proximity_pilot_state = STATE_PP_NOT_CONNECTED;
	} else if (proximity_pilot_voltage == 2.0) {
		proximity_pilot_state = STATE_PP_BUTTON_PRESSED;
	} else if (proximity_pilot_voltage == 1.0) {
		proximity_pilot_state= STATE_PP_CONNECTED;
	}
}

volatile int requested_amps;
volatile float duty = 0;
volatile float frequency = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    	uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t pulse  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    	if (period != 0) {
            duty = ((float)pulse * 100.0f) / period;
            frequency = (float)TIMER_CLOCK / period;

            if (frequency < 995 || frequency > 1005) {
            	control_pilot_state = STATE_CP_ERROR;
            	return;
            }

            if (HAL_GPIO_ReadPin(J1772_PILOT_SWITCH_GPIO_Port,
    				J1772_PILOT_SWITCH_Pin) == GPIO_PIN_RESET) {
            		control_pilot_state = STATE_CP_CONNECTED;
            } else {
            	control_pilot_state = STATE_CP_CHARGING;
            }

            if (duty <= 85) {
                requested_amps = (int)(duty*0.6);
            } else {
                requested_amps = (int)((duty-64)*2.5);
            }
    	}
    }
}

void readControlPilotCurrent() {
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}

void stopReadingControlPilotCurrent() {
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);
}
