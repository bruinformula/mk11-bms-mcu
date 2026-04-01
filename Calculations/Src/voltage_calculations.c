#include "voltage_calculations.h"

float bms_pack_voltage;
float lowest_cell_voltage = INFINITY;
float highest_cell_voltage = -INFINITY;
float voltage_conversions[TOTAL_IC][CELLS_PER_IC];

int lowest_cell_voltage_segment;
int lowest_cell_voltage_cell;

// TODO: ACCOMODATE FOR BROKEN CELL READINGS
void computeAllVoltages(uint8_t tIC, cell_asic *ic) {
	bms_pack_voltage = 0.0f;
	lowest_cell_voltage  = INFINITY;
	highest_cell_voltage = -INFINITY;

	adBms6830_read_cell_voltages(tIC, ic);
	for (size_t i = 0; i < tIC; ++i) {
		for (size_t j = 0; j < CELLS_PER_IC; ++j) {
			float cell_voltage = getVoltage(ic[i].cell.c_codes[j]);
			bms_pack_voltage += cell_voltage;
			voltage_conversions[i][j] = cell_voltage;
			if (cell_voltage < lowest_cell_voltage) {
				lowest_cell_voltage = cell_voltage;

				lowest_cell_voltage_segment = i;
				lowest_cell_voltage_cell = j;
			}

			if (cell_voltage > highest_cell_voltage) {
				highest_cell_voltage = cell_voltage;
			}
		}
	}
}
