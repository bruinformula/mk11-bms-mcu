/*
 * j_plug.h
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#ifndef INC_J_PLUG_H_
#define INC_J_PLUG_H_

// CONTROL PILOT
typedef enum {
	STATE_CP_NOT_CONNECTED,
	STATE_CP_CONNECTED,
	STATE_CP_CHARGING,
	STATE_CP_CHARGING_VENT,
	STATE_CP_ERROR
} STATE_CP;
extern STATE_CP control_pilot_state;

// PROXIMITY PILOT
typedef enum {
	STATE_PP_NOT_CONNECTED,
	STATE_PP_BUTTON_PRESSED,
	STATE_PP_CONNECTED
} STATE_PP;
extern STATE_PP proximity_pilot_state;

#define TIMER_CLOCK 1000000
extern float duty;
extern float frequency;

void readProximityPilot();
void readControlPilotCurrent();

#endif /* INC_J_PLUG_H_ */
