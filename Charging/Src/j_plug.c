/*
 * j_plug.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "j_plug.h"

volatile STATE_CP control_pilot_state = STATE_CP_IDLE;
volatile STATE_PP proximity_pilot_state = STATE_PP_IDLE;

// PROXIMITY PILOT VOLTAGE LEVEL CALCULATIONS HANDLED IN "adc.c" CALLBACK!
volatile uint16_t proximity_pilot_adc;
volatile float proximity_pilot_voltage;

volatile int advertised_amps = 0;
static volatile float duty = 0;
static volatile float frequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// Compute PWM of Control Pilot
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    	uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t pulse  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    	if (period != 0) {
            duty = ((float)pulse * 100.0f) / period;
            frequency = (float)TIMER_CLOCK / period;

            if (frequency < 995 || frequency > 1005) {
            	// frequency should be ~1000
            	control_pilot_state = STATE_CP_ERROR;
            	return;
            }

            if (HAL_GPIO_ReadPin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin) == GPIO_PIN_RESET) {
            		control_pilot_state = STATE_CP_CONNECTED;
            } else {
            	control_pilot_state = STATE_CP_CHARGING;
            }

            if (duty <= 85) {
                advertised_amps = (int)(duty*0.6);
            } else {
                advertised_amps = (int)((duty-64)*2.5);
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
