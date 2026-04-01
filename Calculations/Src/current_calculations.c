/*
 * current_calculations.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#include "current_calculations.h"

// ALL CURRENT SENSOR CALCULATIONS HANDLED IN "adc.h" CALLBACK
uint16_t current_sensor_low_adc;
uint16_t current_sensor_high_adc;

uint16_t pp_raw_adc;

float current_sensor_low;
float current_sensor_high;
float current_sensor_val;
