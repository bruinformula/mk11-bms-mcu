/*
 * bms_state.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_BMS_STATE_H_
#define INC_BMS_STATE_H_

#include "adBms_Application.h"
#include "fdcan.h"
#include "gpio.h"
#include "j_plug.h"
#include "cmsis_os.h"

typedef enum {
	BMS_IDLE = 0,
	BMS_EXTERNAL_FAULT,
	BMS_INTERNAL_FAULT,
	BMS_CHARGING,
	BMS_BALANCING,
	BMS_PRECHARGING,
	BMS_DRIVE,
} BMS_STATE;

extern volatile BMS_STATE bms_state;

void determine_startup_mode();
void wakeup_tasks();
void change_baud_rate_250();

#endif /* INC_BMS_STATE_H_ */
