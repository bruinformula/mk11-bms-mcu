/*
 * charging.c
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#include "charging.h"

CHARGING_STATE charging_state = CHG_IDLE;

void charging_loop() {
	uint32_t events;
    xTaskNotifyWait(0, UINT32_MAX, &events, 50);

    uint16_t pp_adc;
    uint32_t cp_period;
    uint32_t cp_pulse;

    // CRITICAL REGION
    taskENTER_CRITICAL();
    pp_adc = j1772_context.proximity_pilot_adc;
    cp_period = j1772_context.control_pilot_period;
    cp_pulse = j1772_context.control_pilot_pulse;
    last_cp_tick = j1772_context.last_cp_update_tick;
    taskEXIT_CRITICAL();

    proximity_pilot_state = readProximityPilot(pp_adc);
    if (proximity_pilot_state != STATE_PP_CONNECTED) { // Proximity Pilot not connected.
    	control_pilot_state = STATE_CP_NOT_AVAILABLE;
    } else if (isControlPilotTimedOut(last_cp_tick)) {
    	control_pilot_state = STATE_CP_NOT_AVAILABLE; // Control Pilot is idle.
    } else {
    	control_pilot_state = readControlPilot(cp_period, cp_pulse);
    }

    if (events & EVT_ELCON_FAULT) {
    	charging_state = CHG_ELCON_FAULT;
    }

    switch (charging_state) {
    	case CHG_IDLE:
    		HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_RESET);
    		sendChargerRequest(0, 0, 1);
    		if (proximity_pilot_state == STATE_PP_CONNECTED &&
    				control_pilot_state == STATE_CP_PWM_PRESENT) {
    			charging_state = CHG_WAITING;
    		}
    		break;
    	case CHG_WAITING:
    		if (proximity_pilot_state != STATE_PP_CONNECTED ||
    				control_pilot_state != STATE_CP_PWM_PRESENT) {
    			charging_state = CHG_IDLE;
    			break;
    		}
    		HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_SET);
    		charging_state = CHG_ACTIVE;
    		break;

        case CHG_ACTIVE:
        	if (proximity_pilot_state != STATE_PP_CONNECTED ||
        			control_pilot_state != STATE_CP_PWM_PRESENT) {
        		charging_state = CHG_IDLE;
        	    break;
        	}

        	if (voltage_context.highest_cell_voltage > MAX_CELL_VOLTAGE_CHARGING_THRESHOLD) {
        		charging_state = CHG_COMPLETE;
        	} else {
        		sendChargerRequest(CHARGER_VOLTAGE, advertised_amps, 0);
        	}
        	break;

        case CHG_ELCON_FAULT:
			HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_RESET);
			sendChargerRequest(0,0,1);
			break;

        case CHG_COMPLETE:
			HAL_GPIO_WritePin(J1772_PILOT_SWITCH_GPIO_Port, J1772_PILOT_SWITCH_Pin, GPIO_PIN_RESET);
			sendChargerRequest(0,0,1);
			break;
    }
}
