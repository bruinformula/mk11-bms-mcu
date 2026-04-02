#include "balancing.h"
#include "adBms6830GenericType.h"

#include "main.h"

static uint8_t Calculate_Proportional_PWM(int16_t cell_voltage, int16_t target_voltage){
	int16_t delta_codes = cell_voltage - target_voltage;

	if(delta_codes <= BALANCE_THRESHOLD_CODES){
		return PWM_0_0_PCT;
	}

	int32_t delta_mv = (delta_codes * 150) / 1000;

	if (delta_mv >= MAX_DELTA_MV_FOR_100_PCT){
		return PWM_100_0_PCT;
	}

	uint8_t pwm_step = 1 + ((delta_mv - BALANCE_THRESHOLD_MV) * 14) / (MAX_DELTA_MV_FOR_100_PCT - BALANCE_THRESHOLD_MV);
	if (pwm_step > 15){
		pwm_step = 15;
	}
	return pwm_step;
}

void Start_Cell_Balancing(cell_asic *ic, uint8_t total_ic){
	uint8_t is_balanced = 0;

	while (!is_balanced){
		for (int i = 0; i < total_ic; i ++){
			ic[i].tx_cfgb.dcc = 0x0000;
			for(int c=0; c < PWMA; c++) ic[i].PwmA.pwma[c] = PWM_0_0_PCT;

		}
		adBms6830_write_config(total_ic, ic);
		adBms6830_write_read_pwm_duty_cycle(total_ic, ic);

		HAL_Delay(STABILIZE_TIME_MS);
		adBms6830_start_adc_cell_voltage_measurment(total_ic);
		HAL_Delay(10);
		adBms6830_read_cell_voltages(total_ic, ic);
		HAL_Delay(MONITOR_TIME_MS);

		int16_t min_voltage = 32767;
		int16_t max_voltage = -32768;

		for (int i = 0; i < total_ic; i++) {
			for (int c = 0; c < CELLS_PER_IC; c++) {
				int16_t current_cell_v = ic[i].cell.c_codes[c];
				if (current_cell_v < min_voltage) min_voltage = current_cell_v;
				if (current_cell_v > max_voltage) max_voltage = current_cell_v;
			}
		}

		if ((max_voltage - min_voltage) <= BALANCE_THRESHOLD_CODES) {
			is_balanced = 1;
			break;
		}

		for (int i = 0; i < total_ic; i++) {
			uint16_t discharge_mask = 0;

			for (int c = 0; c < CELLS_PER_IC; c++) {
				int16_t current_cell_v = ic[i].cell.c_codes[c];

				uint8_t cell_pwm = Calculate_Proportional_PWM(current_cell_v, min_voltage);

				if (cell_pwm > PWM_0_0_PCT) {
					discharge_mask |= (1 << c);
					ic[i].PwmA.pwma[c] = cell_pwm;

				}
			}
			ic[i].tx_cfgb.dcc = discharge_mask;
		}
	}
	adBms6830_write_config(total_ic, ic);
	adBms6830_write_read_pwm_duty_cycle(total_ic, ic);

	HAL_Delay(DISCHARGE_TIME_MS);

	for (int i = 0; i < total_ic; i++) {
	        ic[i].tx_cfgb.dcc = 0x0000;
	        for(int c=0; c < PWMA; c++) ic[i].PwmA.pwma[c] = PWM_0_0_PCT;
	}
	adBms6830_write_config(total_ic, ic);
	adBms6830_write_read_pwm_duty_cycle(total_ic, ic);
}
