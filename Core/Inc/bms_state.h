/*
 * bms_state.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_BMS_STATE_H_
#define INC_BMS_STATE_H_

#include "gpio.h"
#include "charging.h"
#include "balancing.h"
#include "currLimiting.h"
#include "datalogging.h"
#include "adBms_Application.h"
#include "cmsis_os.h"

typedef enum {
	BMS_IDLE = 0,
	BMS_FAULT,
	BMS_CHARGING,
	BMS_BALANCING,
	BMS_PRECHARGING,
	BMS_DRIVE,
} BMS_STATE;

extern volatile BMS_STATE bms_state;

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void determineStartupMode();
void enterDriveMode();

#endif /* INC_BMS_STATE_H_ */
