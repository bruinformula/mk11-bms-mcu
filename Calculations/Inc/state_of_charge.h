/*
 * state_of_charge.h
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#ifndef INC_STATE_OF_CHARGE_H_
#define INC_STATE_OF_CHARGE_H_

#include <stdint.h>
#include "thermistor.h"
#include "voltage_calculations.h"
#include "ocv_table.h"

#define NOMINAL_PACK_CAPACITY_AH 14.414
#define NOMINAL_PACK_CAPACITY_AS (NOMINAL_PACK_CAPACITY_AH*3600.0f)
#define NOMINAL_PACK_VOLTAGE 420

extern float soc;

void get_initial_soc();
void coulomb_count(uint32_t dt_ms);

#endif /* INC_STATE_OF_CHARGE_H_ */
