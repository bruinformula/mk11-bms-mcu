/*
 * thermistor.c
 *
 *  Created on: Feb 10, 2026
 *      Author: ishanchitale
 */

#include "thermistor.h"

const float voltage_table[33] = {
		2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.8, 1.74, 1.68,
		1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30
};

const float temp_table[33] = {
		-40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
		55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120
};

float temp_conversions[10];

float voltageToTemp(float V) {
	if (V > voltage_table[0] || V < voltage_table[32]) {
		return 999.0; // Out of range
	}

	for (size_t i = 0; i < 32; i++) {
		float v_high = voltage_table[i];      // higher voltage, lower temp
		float v_low = voltage_table[i + 1];   // lower voltage, higher temp

		if (V <= v_high && V >= v_low) {
			float t_high = temp_table[i];
			float t_low = temp_table[i + 1];

			// LINEAR INTERPOLATION
			float temp = t_high + (V - v_high) * (t_low - t_high) / (v_low - v_high);
			return temp;
		}
	}

	return 999.0;
}

void computeAllTemps(uint8_t tIC, cell_asic *ic) {
	adBms6830_read_aux_voltages(TOTAL_IC, IC);
	for (size_t i = 0; i < 10; ++i) {
		temp_conversions[i] = voltageToTemp(getVoltage(IC->aux.a_codes[i]));
	}
}
