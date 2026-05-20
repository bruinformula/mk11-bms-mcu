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
#include "prchg.h"
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
extern volatile BMS_STATE bms_state;

void determine_operating_state();
void reset_operating_state(BMS_STATE prev_state);
void processGUI_Cmd();
void wakeup_tasks();
void change_baud_rate_500();
void change_baud_rate_250();

void enter_wait_for_gui_mode();
void enter_precharge_mode();
void enter_drive_mode();
void enter_charging_mode();
void enter_balancing_mode();

void exit_wait_for_gui_mode();
void exit_precharge_mode();
void exit_drive_mode();
void exit_charging_mode();
void exit_balancing_mode();


#endif /* INC_BMS_STATE_H_ */
