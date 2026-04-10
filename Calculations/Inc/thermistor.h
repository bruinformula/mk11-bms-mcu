/*
 * thermistor.h
 *
 *  Created on: Feb 10, 2026
 *      Author: ishanchitale
 */

#ifndef INC_THERMISTOR_H_
#define INC_THERMISTOR_H_

#include <math.h>
#include "adBms_Application.h"
#include "serialPrintResult.h"
#include "bms_state.h"

#define MAX_VALID_CELL_T 40.0 // temporary value
#define MIN_VALID_CELL_T 10.0 // temporary value

typedef struct {
    uint8_t ic_index;
    uint8_t cell_index;
    float measured_temp;
} BrokenCell_T;

extern bool cell_is_broken_t[TOTAL_IC][CELLS_PER_IC];
extern BrokenCell_T broken_cells_t[TOTAL_CELLS];
extern size_t broken_cell_count_t;

extern float avg_cell_temp;
extern float lowest_cell_temp;
extern float highest_cell_temp;

extern const float voltage_table[33];
extern const float temp_table[33];
extern float temp_conversions[TOTAL_IC][CELLS_PER_IC];

float voltageToTemp(float V);
void computeAllTemps(uint8_t tIC, cell_asic *ic);

#endif /* INC_THERMISTOR_H_ */
