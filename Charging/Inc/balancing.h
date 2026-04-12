/*
 * balancing.h
 *
 *  Created on: Feb 28, 2026
 *      Author: ishanchitale
 */

#ifndef INC_BALANCING_H_
#define INC_BALANCING_H_

#include "current_calculations.h"
#include "voltage_calculations.h"
#include "adBms_Application.h"
#include "adBms6830GenericType.h"
#include "adBms6830CmdList.h"
#include "cmsis_os.h"
#include "freertos_handles.h"

#define BALANCE_VOLTAGE_THRESHOLD 0.001
#define BALANCE_BLEED_PERIOD 60000
#define BALANCE_WAIT_PERIOD 60000

typedef enum {
	BALANCE_COMPUTE_DISCHARGE,
	BALANCE_DISCHARGE,
	BALANCE_WAIT,
	BALANCE_COMPLETE
} BalanceState;

void fastBalancingLoop(uint8_t tIC, cell_asic *ic);
void pwmBalancingLoop(uint8_t tIC, cell_asic *ic); // TODO, for <100% DISCHARGE CURRENT

#endif /* INC_BALANCING_H_ */
