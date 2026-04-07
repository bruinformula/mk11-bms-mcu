/*
 * balancing.c
 *
 *  Created on: Feb 28, 2026
 *      Author: ishanchitale
 */

#include "balancing.h"

static int num_unbalanced_cells = TOTAL_CELLS;
static BalanceState balance_state = BALANCE_IDLE;

void fastBalancingLoop(uint8_t tIC, cell_asic *ic) {
	// SETUP: SET DCTO (Discharge Timeout) to nonzero value
	for (size_t i = 0; i < tIC; ++i) {
		ic[i].tx_cfgb.dcto = NONZERO_DCTO;
	}
	adBms6830_write_config(tIC, ic);
	adBms6830_read_config(tIC, ic);

	while (num_unbalanced_cells > 0) {
		// RECOMPUTE VOLTAGES
		computeAllVoltages(tIC, IC);
		adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_ON, RSTF_OFF, OW_OFF_ALL_CH);
		num_unbalanced_cells = 0;

		// CALCULATE DCC BITMASK
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

		// DISCHARGE FOR 60 SECONDS (BALANCE_BLEED_PERIOD)
		balance_state = BALANCE_DISCHARGE;
		uint32_t discharge_start_time = HAL_GetTick();
		while (HAL_GetTick() - discharge_start_time <= BALANCE_BLEED_PERIOD) {
			adBms6830_write_config(tIC, ic);
			HAL_Delay(500);
		}

		// WAIT FOR 60 SECONDS (BALANCE_WAIT_PERIOD)
		for (size_t i = 0; i < tIC; ++i) {
			ic[i].tx_cfgb.dcc = 0x0000;
		}
		balance_state = BALANCE_WAIT;
		uint32_t wait_period_start_time = HAL_GetTick();
		while (HAL_GetTick() - wait_period_start_time <= BALANCE_WAIT_PERIOD) {
			adBms6830_write_config(tIC, ic);
			HAL_Delay(500);
		}

	}
}
