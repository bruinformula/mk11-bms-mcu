/*
 * state_of_charge.h
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#ifndef INC_VOLTAGE_CALCULATIONS_H_
#define INC_VOLTAGE_CALCULATIONS_H_

#include "adBms_Application.h"
#include "serialPrintResult.h"
#include "safety_handler.h"
#include "cmsis_os.h"
#include "freertos_handles.h"
#include <math.h>

#define OVER_VOLTAGE_THRESHOLD 4.20
#define UNDER_VOLTAGE_THRESHOLD 2.40
#define BROKEN_CELL_VOLTAGE_THRESHOLD 1.60

typedef struct VOLTAGE_CONTEXT {
	float estimated_pack_voltage;
	float lowest_cell_voltage;
	int lowest_cell_ic;
	int lowest_cell_index;
	float avg_cell_voltage;
	float highest_cell_voltage;
	int highest_cell_ic;
	int highest_cell_index;
	float voltage_conversions[TOTAL_IC][CELLS_PER_IC];
	int num_valid_cell_voltages;
} VOLTAGE_CONTEXT;
extern volatile VOLTAGE_CONTEXT voltage_context;

void computeAllVoltages(uint8_t tIC, cell_asic *ic);

#endif /* INC_VOLTAGE_CALCULATIONS_H_ */
