/*
 * bms_state.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "bms_state.h"

volatile BMS_STATE bms_state = BMS_IDLE;

#define RX_BUF_SIZE 32
static uint8_t uart_rx_buffer[RX_BUF_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t len) {
	if (huart->Instance == LPUART1) {
		char cmd[RX_BUF_SIZE];
		uint16_t copy_len = (len < RX_BUF_SIZE - 1) ? len : (RX_BUF_SIZE - 1);
		memcpy(cmd, uart_rx_buffer, copy_len);
		cmd[copy_len] = '\0';

		char *newline = strchr(cmd, '\n');
		if (newline) *newline = '\0';
		char *carriage = strchr(cmd, '\r');
		if (carriage) *carriage = '\0';

		if (strcmp(cmd, "ENTER_CHG_MODE") == 0) {
			enter_charging_mode();
		} else if (strcmp(cmd, "EXIT_CHG_MODE") == 0) {
			// TODO
		} else if (strcmp(cmd, "ENTER_BAL_MODE") == 0) {
			enter_balancing_mode();
		} else if (strcmp(cmd, "EXIT_BAL_MODE") == 0) {
			// TODO
		} else if (strncmp(cmd, "START_BAL", 10) == 0) {
			// Parse the % sent in the serial command from GUI.
			startBalancingLoop(atoi(&cmd[10]));
		}

        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rx_buffer, RX_BUF_SIZE);
	}
}

void determine_operating_state() {
	if (HAL_GPIO_ReadPin(CHARGE_SIGNAL_GPIO_Port, CHARGE_SIGNAL_Pin) == GPIO_PIN_SET) {
		bms_state = BMS_WAIT_FOR_GUI;
		// Receive commands over Serial (GUI)!
		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rx_buffer, RX_BUF_SIZE);
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		enter_precharge_mode();
	}
}

void reset_operating_state(BMS_STATE prev_state) {
    switch (prev_state) {
    	case BMS_CHARGING:
    		exit_charging_mode();
    	case BMS_BALANCING:
    		exit_balancing_mode();
    	case BMS_PRECHARGING:
    		exit_precharge_mode();
    	case BMS_DRIVE:
    		exit_drive_mode();
    	default:
    		// BMS_INTERNAL_FAULT or BMS_EXTERNAL_FAULT
    		// Should never reach here!
    		break;
    }
}

void wakeup_tasks() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	switch(bms_state) {
		case BMS_BALANCING:
			xTaskNotifyFromISR(balancingTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
			break;
		case BMS_CHARGING:
			xTaskNotifyFromISR(chargingTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
			break;
		case BMS_PRECHARGING:
			xTaskNotifyFromISR(prchgTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
			break;
		case BMS_DRIVE:
            xTaskNotifyFromISR(currLimitTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
            break;

		default:
			break;
	}

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void change_baud_rate_500() {
    HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	HAL_FDCAN_Init(&hfdcan1);
	startCAN_Tx_Rx();
}

void change_baud_rate_250() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON CHARGER!
    HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
	HAL_FDCAN_Init(&hfdcan1);
	startCAN_Tx_Rx();
}

// ENTER BMS STATES!
void enter_precharge_mode() {
	bms_state = BMS_PRECHARGING;
	precharge_state = PRECHARGE_IDLE;
	wakeup_tasks();
}

void enter_drive_mode() {
	bms_state = BMS_DRIVE;
	// No specialized state machine for driving.
	// prechsrge_state should be PRECHARGE_SUCCESS.
	wakeup_tasks();
}

void enter_charging_mode() {
	// Close AIRs to begin charging!
	HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_SET);

	change_baud_rate_250(); // To read ELCON Charger.
	startPWM_Capture(); // To read Control Pilot.

	bms_state = BMS_CHARGING;
	charging_state = CHG_IDLE;
	wakeup_tasks();
}

void enter_balancing_mode() {
	// DCTO must be set non-zero.
	for (size_t i = 0; i < TOTAL_IC; ++i) {
		IC[i].tx_cfgb.dcto = 7;
	}
	adBms6830_write_config(TOTAL_IC, IC);

	bms_state = BMS_BALANCING;
	balance_state = BALANCE_IDLE;
	wakeup_tasks();
}

// EXIT BMS STATES!
// Reset any specialized state machines and associated necessary variables.
void exit_precharge_mode() {
	precharge_state = PRECHARGE_IDLE;
	bms_state = BMS_IDLE;
}

void exit_drive_mode() {
	// NOTE: No specialized state machine associated with driving.
	// However, successful precharge is necessary for transition to driving.
	// Thus, we effectively are exiting precharge mode.
	precharge_state = PRECHARGE_IDLE;
	bms_state = BMS_IDLE;
}

void exit_charging_mode() {
	charging_state = CHG_IDLE;
	HAL_UART_DMAStop(&hlpuart1); // Prevent GUI Commands.
    // change_baud_rate_500();
	// Keep Baud Rate at 250 Kbps for debugging reasons.
	// Still want to use CAN even if we are not actively charging.

     stopPWM_Capture();
	// Control Pilot readings not necessary if charging is not active.

	bms_state = BMS_IDLE;
}

void exit_balancing_mode() {
	balance_state = BALANCE_IDLE;
	HAL_UART_DMAStop(&hlpuart1); // Prevent GUI Commands.
	// Reset Discharge Current % to 0.
	for (size_t i = 0; i < TOTAL_IC; ++i) {
		memset(IC[i].PwmA.pwma, 0x00, sizeof(IC[i].PwmA.pwma));
		memset(IC[i].PwmB.pwmb, 0x00, sizeof(IC[i].PwmB.pwmb));
	}
	// Reset DCTO to zero.
	for (size_t i = 0; i < TOTAL_IC; ++i) {
		IC[i].tx_cfgb.dcto = 0;
	}
	adBms6830_write_config(TOTAL_IC, IC);
	bms_state = BMS_IDLE;
}
