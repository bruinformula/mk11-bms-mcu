#include "voltage_calculations.h"

float bms_pack_voltage;
float lowest_cell_voltage = INFINITY;
float highest_cell_voltage = -INFINITY;
float avg_cell_voltage = 0;
float voltage_conversions[TOTAL_IC][CELLS_PER_IC];

// TODO: ACCOMODATE FOR BROKEN CELL VOLTAGE READINGS, THINK ABOUT RACE CONDITIONS
void computeAllVoltages(uint8_t tIC, cell_asic *ic) {

	avg_cell_voltage = 0.0f;
	float temp_bms_pack_voltage = 0.0f;
	lowest_cell_voltage  = INFINITY;
	highest_cell_voltage = -INFINITY;

	adBms6830_read_cell_voltages(tIC, ic);

	for (size_t i = 0; i < tIC; ++i) {
		for (size_t j = 0; j < CELLS_PER_IC; ++j) {
			float cell_voltage = getVoltage(ic[i].cell.c_codes[j]);
			temp_bms_pack_voltage += cell_voltage;
			voltage_conversions[i][j] = cell_voltage;
			if (cell_voltage < lowest_cell_voltage) {
				lowest_cell_voltage = cell_voltage;
			}

			if (cell_voltage > highest_cell_voltage) {
				highest_cell_voltage = cell_voltage;
			}
		}
	}
	avg_cell_voltage = temp_bms_pack_voltage/(TOTAL_CELLS);
	bms_pack_voltage = temp_bms_pack_voltage;
}
