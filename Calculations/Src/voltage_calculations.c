#include "voltage_calculations.h"

float bms_pack_voltage;
float lowest_cell_voltage = INFINITY;
float highest_cell_voltage = -INFINITY;
float avg_cell_voltage = 0;
float voltage_conversions[TOTAL_IC][CELLS_PER_IC];


bool cell_is_broken_v[TOTAL_IC][CELLS_PER_IC];
BrokenCell_V broken_cells_v[TOTAL_CELLS];
size_t broken_cell_count_v = 0;

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
			if (cell_voltage > MAX_VALID_CELL_V || cell_voltage < MIN_VALID_CELL_V) {

				cell_is_broken_v[i][j] = true;
				if (broken_cell_count_v < TOTAL_CELLS) {
					broken_cells_v[broken_cell_count_v].cell_index = j;
					broken_cells_v[broken_cell_count_v].ic_index = i;
					broken_cells_v[broken_cell_count_v].measured_voltage = cell_voltage;
					broken_cell_count_v++;
				}

				voltage_conversions[i][j] = NAN;
				continue;
			}

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

	if (TOTAL_CELLS - broken_cell_count_v == 0) avg_cell_voltage = -1.0;
	else avg_cell_voltage = temp_bms_pack_voltage/(TOTAL_CELLS - broken_cell_count_v);

	bms_pack_voltage = temp_bms_pack_voltage;
}
