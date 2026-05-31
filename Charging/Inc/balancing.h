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

#define BALANCE_VOLTAGE_THRESHOLD 0.005
#define BALANCE_BLEED_PERIOD 600000
#define BALANCE_WAIT_PERIOD 20000

typedef enum {
	BALANCE_IDLE,
	BALANCE_COMPUTE_DISCHARGE,
	BALANCE_DISCHARGE,
	BALANCE_WAIT,
	BALANCE_STOP_REQUESTED,
	BALANCE_COMPLETE
} BalanceState;
extern volatile BalanceState balance_state;

void set_cell_pwm(cell_asic* ic, uint8_t ic_num, uint8_t cell_num);
void startBalancingLoop(int balance_percent);
void stopBalancingLoop(uint8_t tIC, cell_asic *ic);
void balancingLoop(uint8_t tIC, cell_asic *ic);
uint8_t getBalancePercent(void);

#endif /* INC_BALANCING_H_ */
