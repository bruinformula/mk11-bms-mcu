/*
 * j_plug.h
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#ifndef INC_J_PLUG_H_
#define INC_J_PLUG_H_

#include <stdint.h>
#include <math.h>
#include "adc.h"
#include "tim.h"

#define PP_VOLTAGE_EPSILON 0.1
#define TIMER_CLOCK 1000000

// J1772 CONTROL PILOT
typedef enum {
	STATE_CP_ERROR,
	STATE_CP_PWM_PRESENT, // J1772 Connected, 1 Khz PWM Present; actual charging dependent on J1772_PILOT_SWITCH State
} STATE_CP;
extern volatile STATE_CP control_pilot_state;

// J1772 PROXIMITY PILOT
typedef enum {
	STATE_PP_NOT_CONNECTED, // J1772 not connected
	STATE_PP_BUTTON_PRESSED, // J1772 latch button pressed, temporarily stops charging
	STATE_PP_CONNECTED // J1772 connected
} STATE_PP;
extern volatile STATE_PP proximity_pilot_state;

typedef struct J1772_CONTEXT {
	volatile STATE_CP control_pilot_state;
	volatile int control_pilot_advertised_amps;
	volatile float control_pilot_freq;
	volatile float control_pilot_duty_cycle;
	volatile STATE_PP proximity_pilot_state;
	volatile uint16_t proximity_pilot_adc;
	volatile float proximity_pilot_voltage;
} J1772_CONTEXT;

extern J1772_CONTEXT j1772_context;

void readControlPilot();
void stopReadingControlPilot();
void readProximityPilot();

#endif /* INC_J_PLUG_H_ */
