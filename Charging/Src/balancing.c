/*
 * balancing.c
 *
 *  Created on: Feb 28, 2026
 *      Author: ishanchitale
 */

#include "balancing.h"

static float voltage_conversions_snapshot[TOTAL_IC][CELLS_PER_IC];
static float local_lowest_cell_voltage;
static int num_unbalanced_cells;
static uint32_t phase_start_time;
static BalanceState balance_state = BALANCE_COMPUTE_DISCHARGE;

void fastBalancingLoop(uint8_t tIC, cell_asic *ic) {
	switch (balance_state) {
		case BALANCE_COMPUTE_DISCHARGE:
			// CRITICAL REGION
			osMutexWait(VOLTAGE_MUTEXHandle, osWaitForever);
			memcpy(voltage_conversions_snapshot, voltage_context.voltage_conversions, sizeof(voltage_conversions_snapshot));
			local_lowest_cell_voltage = voltage_context.lowest_cell_voltage;
			osMutexRelease(VOLTAGE_MUTEXHandle);

            num_unbalanced_cells = 0;
            for (size_t i = 0; i < tIC; ++i) {
                uint16_t dcc_mask = 0x0000;
                for (size_t j = 0; j < CELLS_PER_IC; ++j) {
                    if (voltage_conversions_snapshot[i][j] > local_lowest_cell_voltage + BALANCE_VOLTAGE_THRESHOLD) {
                        dcc_mask |= (1 << j);
                        num_unbalanced_cells++;
                    }
                }

                ic[i].tx_cfgb.dcc = dcc_mask;
            }

            if (num_unbalanced_cells > 0) {
                balance_state = BALANCE_DISCHARGE;
                phase_start_time = HAL_GetTick();
            } else {
            	balance_state = BALANCE_COMPLETE;
            }
            break;

		case BALANCE_DISCHARGE:
			// CRITICAL REGION
			osMutexWait(SPI_MUTEXHandle, osWaitForever);
            adBms6830_write_config(tIC, ic);
            osMutexRelease(SPI_MUTEXHandle);

            if (HAL_GetTick() - phase_start_time >= BALANCE_BLEED_PERIOD) {
                for (size_t i = 0; i < tIC; ++i) {
                    ic[i].tx_cfgb.dcc = 0x0000;
                }
                balance_state = BALANCE_WAIT;
                phase_start_time = HAL_GetTick();
            }
            break;

		case BALANCE_WAIT:
			// CRITICAL REGION
			osMutexWait(SPI_MUTEXHandle, osWaitForever);
            adBms6830_write_config(tIC, ic);
            osMutexRelease(SPI_MUTEXHandle);

            if (HAL_GetTick() - phase_start_time >= BALANCE_WAIT_PERIOD) {
                balance_state = BALANCE_COMPUTE_DISCHARGE;
            }
            break;

		case BALANCE_COMPLETE:
			// DO NOTHING, BLOCK
			break;

	}
}
