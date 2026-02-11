/*
 * thermistor.h
 *
 *  Created on: Feb 10, 2026
 *      Author: ishanchitale
 */

#ifndef INC_THERMISTOR_H_
#define INC_THERMISTOR_H_

#include "adBms_Application.h"
#include "serialPrintResult.h"

extern const float voltage_table[33];
extern const float temp_table[33];
extern float temp_conversions[10];

float voltageToTemp(float V);
void computeAllTemps(uint8_t tIC, cell_asic *ic);

#endif /* INC_THERMISTOR_H_ */
