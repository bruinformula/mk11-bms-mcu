/*
 * bms_state.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_BMS_STATE_H_
#define INC_BMS_STATE_H_

#include <can_datalogging.h>
#include <curr_limiting.h>
#include "gpio.h"
#include "charging.h"
#include "balancing.h"
#include "adBms_Application.h"

typedef enum {
	BMS_IDLE = 0,
	BMS_SHUTDOWN_FAULT,
	BMS_INTERNAL_FAULT,
	BMS_CHARGING,
	BMS_BALANCING,
	BMS_PRECHARGING,
	BMS_DRIVE,
} BMS_STATE;

extern volatile BMS_STATE bms_state;

void determineStartupMode();

#endif /* INC_BMS_STATE_H_ */
