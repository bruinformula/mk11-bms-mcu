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
#include "usart.h"
#include "charging.h"
#include "balancing.h"
#include "adBms_Application.h"

typedef enum {
	BMS_IDLE = 0,
	BMS_INTERNAL_FAULT,
	BMS_EXTERNAL_FAULT,
	BMS_WAIT_FOR_GUI,
	BMS_CHARGING,
	BMS_BALANCING,
	BMS_PRECHARGING,
	BMS_DRIVE,
} BMS_STATE;

extern bool shutdown_power;
extern volatile BMS_STATE bms_state;

void determine_startup_mode();
void enter_precharge_mode();
void enter_drive_mode();
void enter_charging_mode();
void enter_balancing_mode();
void wakeup_tasks();
void change_baud_rate_250();

#endif /* INC_BMS_STATE_H_ */
