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
#include "freertos_handles.h"
#include "cmsis_os.h"

typedef enum {
	BMS_IDLE = 0,
	BMS_FAULT,
	BMS_CHARGING,
	BMS_BALANCING,
	BMS_PRECHARGING,
	BMS_DRIVE,
} BMS_STATE;

extern BMS_STATE bms_state;

void determineStartupMode();
void enterDriveMode();
void spi_lock();
void spi_unlock();

#endif /* INC_BMS_STATE_H_ */
