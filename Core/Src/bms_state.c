/*
 * bms_state.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "bms_state.h"

volatile BMS_STATE bms_state = BMS_IDLE;

void determineStartupMode() {
	if (HAL_GPIO_ReadPin(CHARGE_SIGNAL_GPIO_Port, CHARGE_SIGNAL_Pin) == GPIO_PIN_SET) {
		if (HAL_GPIO_ReadPin(BALANCING_EN_GPIO_Port, BALANCING_EN_Pin) == GPIO_PIN_SET) {
			bms_state = BMS_BALANCING;
			// DCTO must be set non-zero.
			for (size_t i = 0; i < TOTAL_IC; ++i) {
				IC[i].tx_cfgb.dcto = 5;
			}
			adBms6830_write_config(TOTAL_IC, IC);
		} else {
			bms_state = BMS_CHARGING;
			change_baud_rate(); // Change to 250 Kbps!
			readControlPilot();
		}
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		bms_state = BMS_PRECHARGING;
	}
}
