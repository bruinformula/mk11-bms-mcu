#include "state_of_charge.h"

static float interpolate(float x0, float x1, float y0, float y1, float x) {
	if (x0 == x1) return y0;
	return y0 + (x-x0)*(y1-y0)/(x1-x0);
}

// TODO: FAULT HANDLING FOR OUT OF RANGE VALUES
static float ocv_lookup_soc(float avg_temp, float cell_voltage) {
	int t_low = 0;
	int t_high = 0;

	if (avg_temp < temp_axis[0] || avg_temp > temp_axis[NUM_TEMP-1]) {
		return -1;
	}

	// GET SURROUNDING TEMPERATURES
	for (size_t i = 0; i < NUM_TEMP-1; ++i) {
		if (avg_temp >= temp_axis[i] && avg_temp <= temp_axis[i+1]) {
			t_low = i;
			t_high = i+1;
			break;
		}
	}

	// CREATE SOC CURVE
	float ocv_interp[NUM_SOC];
	for (size_t j = 0; j < NUM_SOC; j++) {
		ocv_interp[j] = interpolate(temp_axis[t_low],
				temp_axis[t_high],
				ocv_table[t_low][j],
				ocv_table[t_high][j],
				avg_temp);
	}

	if (cell_voltage < ocv_interp[0] || cell_voltage > ocv_interp[NUM_SOC-1]) {
		return -1;
	}

	// INTERPOLATE
	for (size_t k = 0; k < NUM_SOC-1; ++k) {
		if (cell_voltage >= ocv_interp[k] && cell_voltage <= ocv_interp[k+1]) {
			float soc = interpolate(ocv_interp[k],
					ocv_interp[k+1],
					soc_axis[k],
					soc_axis[k+1],
					cell_voltage);

			return soc;
		}
	}

	return -1;
}

float soc;
void get_initial_soc() {
	float soc_min = ocv_lookup_soc(avg_cell_temp, lowest_cell_voltage);
	float soc_max = ocv_lookup_soc(avg_cell_temp, highest_cell_voltage);

	if (soc_min <= 50.0) {
		soc = soc_min;
	} else {
		soc = soc_max;
	}
}

void coulomb_count(uint32_t dt_ms) {
	float dt_seconds = (float)dt_ms / 1000.0f;
	float delta_soc = (current_sensor_val * dt_seconds)/NOMINAL_PACK_CAPACITY_AS;
	delta_soc*=100.0f;

	// Discharge or Charge?
	soc -= delta_soc;
}
