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

// CONTROL PILOT
typedef enum {
	STATE_CP_IDLE,
	STATE_CP_CONNECTED,
	STATE_CP_CHARGING,
	STATE_CP_ERROR,
} STATE_CP;
extern volatile STATE_CP control_pilot_state;

// PROXIMITY PILOT
typedef enum {
	STATE_PP_IDLE,
	STATE_PP_NOT_CONNECTED,
	STATE_PP_BUTTON_PRESSED,
	STATE_PP_CONNECTED
} STATE_PP;
extern volatile STATE_PP proximity_pilot_state;

extern volatile uint16_t proximity_pilot_adc;
extern volatile float proximity_pilot_voltage;
extern volatile int advertised_amps;

void readControlPilot();
void stopReadingControlPilot();

#endif /* INC_J_PLUG_H_ */
