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
#include "bms_state.h"

#define MAX_VALID_CELL_V 5.0
#define MIN_VALID_CELL_V 0.1

typedef struct {
    uint8_t ic_index;
    uint8_t cell_index;
    float measured_voltage;
} BrokenCell_V;

extern bool cell_is_broken_v[TOTAL_IC][CELLS_PER_IC];
extern BrokenCell_V broken_cells_v[TOTAL_CELLS];
extern size_t broken_cell_count_v;

extern float bms_pack_voltage;
extern float lowest_cell_voltage;
extern float highest_cell_voltage;
extern float avg_cell_voltage;
extern float voltage_conversions[TOTAL_IC][CELLS_PER_IC];
void computeAllVoltages(uint8_t tIC, cell_asic *ic);

#endif /* INC_VOLTAGE_CALCULATIONS_H_ */
