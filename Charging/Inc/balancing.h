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

#define BALANCE_VOLTAGE_THRESHOLD 0.05
#define BALANCE_BLEED_PERIOD 60000
#define BALANCE_WAIT_PERIOD 60000

void balancingLoop(uint8_t tIC, cell_asic *ic);

#endif /* INC_BALANCING_H_ */
