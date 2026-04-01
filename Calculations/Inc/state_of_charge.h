/*
 * state_of_charge.h
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#ifndef INC_STATE_OF_CHARGE_H_
#define INC_STATE_OF_CHARGE_H_

#include "thermistor.h"
#include "voltage_calculations.h"
#include "ocv_table.h"

#define NOMINAL_PACK_CAPACITY 14.414
#define NOMINAL_PACK_CAPACITY_C (NOMINAL_PACK_CAPACITY*3600.0f)
#define NOMINAL_PACK_VOLTAGE 420

extern float initial_soc;

float interpolate(float x0, float x1, float y0, float y1, float x);
float ocv_lookup_soc(float avg_temp, float cell_voltage);
void get_initial_soc();
void coulumb_count();

#endif /* INC_STATE_OF_CHARGE_H_ */
