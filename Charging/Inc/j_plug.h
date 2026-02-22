/*
 * j_plug.h
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#ifndef INC_J_PLUG_H_
#define INC_J_PLUG_H_

// CONTROL PILOT
// TODO: IMPLEMENT CONTROL PILOT STATE HANDLING
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

#define TIMER_CLOCK 1000000
extern volatile float duty;
extern volatile float frequency;
extern volatile int requested_amps;
extern volatile float proximity_pilot_voltage;

void readProximityPilot();
void readControlPilotCurrent();
void stopReadingControlPilotCurrent();

#endif /* INC_J_PLUG_H_ */
