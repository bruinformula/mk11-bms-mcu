#include "voltage_calculations.h"

float bms_pack_voltage;
float lowest_cell_voltage = INFINITY;
float highest_cell_voltage = -INFINITY;
float voltage_conversions[TOTAL_IC][CELLS_PER_IC];

void computeAllVoltages(uint8_t tIC, cell_asic *ic) {
	adBms6830_read_cell_voltages(tIC, ic);
	for (size_t i = 0; i < TOTAL_IC; ++i) {
		for (size_t j = 0; j < CELLS_PER_IC; ++j) {
			float cell_voltage = getVoltage(ic[i].cell.c_codes[j]);
			bms_pack_voltage += cell_voltage;
			voltage_conversions[i][j] = cell_voltage;
			if (cell_voltage < lowest_cell_voltage) {
				lowest_cell_voltage = cell_voltage;
			}

			if (cell_voltage > highest_cell_voltage) {
				highest_cell_voltage = cell_voltage;
			}
		}
	}
}
