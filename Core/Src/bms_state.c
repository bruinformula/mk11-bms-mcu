/*
 * bms_state.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "bms_state.h"

volatile BMS_STATE bms_state = BMS_IDLE;

void determine_startup_mode() {
	HAL_FDCAN_DeInit(&hfdcan1);

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
			change_baud_rate_250(); // Change CAN Baud Rate to 250 Kbps via Clock Prescaler.
			startPWM_Capture();
		}
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		bms_state = BMS_PRECHARGING;
	}

	HAL_FDCAN_Init(&hfdcan1);
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sStdFilter);
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sExtFilter);
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void wakeup_tasks() {
	switch(bms_state) {
		case BMS_BALANCING:
			xTaskNotifyGive(balancingTaskHandle);
			break;
		case BMS_CHARGING:
			xTaskNotifyGive(chargingTaskHandle);
			break;
		case BMS_PRECHARGING:
			xTaskNotifyGive(prchgTaskHandle);
			break;
		case BMS_DRIVE:
            xTaskNotifyGive(currLimitTaskHandle);
            break;

		default:
			break;
	}
}

void change_baud_rate_250() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON CHARGER
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
}
