/*
 * state_of_charge.h
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#ifndef INC_VOLTAGE_CALCULATIONS_H_
#define INC_VOLTAGE_CALCULATIONS_H_

#include <math.h>
#include "adBms_Application.h"
#include "serialPrintResult.h"

extern float bms_pack_voltage;
extern float lowest_cell_voltage;
extern float highest_cell_voltage;
extern float voltage_conversions[TOTAL_IC][CELLS_PER_IC];
void computeAllVoltages(uint8_t tIC, cell_asic *ic);

#endif /* INC_VOLTAGE_CALCULATIONS_H_ */
