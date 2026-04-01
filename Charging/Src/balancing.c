/*
 * balancing.c
 *
 *  Created on: Feb 28, 2026
 *      Author: ishanchitale
 */

#include "balancing.h"

static int num_unbalanced_cells = TOTAL_CELLS;

// TODO: Mess with the PWM?
void balancingLoop(uint8_t tIC, cell_asic *ic) {
	while (num_unbalanced_cells > 0) {
		computeAllVoltages(tIC, ic);
		num_unbalanced_cells = 0;

		for (size_t i = 0; i < tIC; ++i) {
			uint16_t dcc_mask = 0x0000;
			for (size_t j = 0; j < CELLS_PER_IC; ++j) {
				if (voltage_conversions[i][j] > lowest_cell_voltage + BALANCE_VOLTAGE_THRESHOLD) {
					num_unbalanced_cells+=1;
					dcc_mask |= (1 << j);
				}
			}
			ic[i].tx_cfgb.dcc = dcc_mask;
		}

		adBms6830_write_config(tIC, ic);
		adBms6830_read_config(tIC, ic);
		HAL_Delay(BALANCE_BLEED_PERIOD);

		for (size_t i = 0; i < tIC; ++i) {
			ic[i].tx_cfgb.dcc = 0x0000;
		}
		adBms6830_write_config(tIC, ic);
		adBms6830_read_config(tIC, ic);
		HAL_Delay(BALANCE_WAIT_PERIOD);
	}
}
