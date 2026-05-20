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
static char serial_cmd_buffer[RX_BUF_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t len) {
	if (huart->Instance == LPUART1) {
		uint16_t copy_len = (len < RX_BUF_SIZE - 1) ? len : (RX_BUF_SIZE - 1);
		memcpy(serial_cmd_buffer, uart_rx_buffer, copy_len);
	    serial_cmd_buffer[copy_len] = '\0';

	    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	    xTaskNotifyFromISR(serialCmdTaskHandle, 0, eIncrement, &xHigherPriorityTaskWoken);

	    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rx_buffer, RX_BUF_SIZE);
	    __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);

	    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void processGUI_Cmd() {
	char *newline = strchr(serial_cmd_buffer, '\n');
	if (newline) *newline = '\0';

	char *carriage = strchr(serial_cmd_buffer, '\r');
	if (carriage) *carriage = '\0';

	if (bms_state == BMS_WAIT_FOR_GUI) {
		if (strcmp(serial_cmd_buffer, "ENTER_CHG_MODE") == 0) {
			enter_charging_mode();
	    } else if (strcmp(serial_cmd_buffer, "ENTER_BAL_MODE") == 0) {
	    	enter_balancing_mode();
	    }
	} else if (bms_state == BMS_CHARGING) {
		// TODO: UNTESTED
		if (strcmp(serial_cmd_buffer, "EXIT_CHG_MODE") == 0) {
			exit_charging_mode();
		}
	} else if (bms_state == BMS_BALANCING) {
		// TODO: Untested
		if (strcmp(serial_cmd_buffer, "EXIT_BAL_MODE") == 0) {
			exit_balancing_mode();
		} else if (strncmp(serial_cmd_buffer, "START_BAL", 9) == 0) {
			int percent = atoi(&serial_cmd_buffer[10]);
			startBalancingLoop(percent);
		}
	}
}

void determine_operating_state() {
	// Based on Power Signal Logic.
	if (HAL_GPIO_ReadPin(CHARGE_SIGNAL_GPIO_Port, CHARGE_SIGNAL_Pin) == GPIO_PIN_SET) {
		enter_wait_for_gui_mode();
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		enter_precharge_mode();
	}
}

void reset_operating_state(BMS_STATE prev_state) {
    switch (prev_state) {
    	case BMS_WAIT_FOR_GUI:
    		exit_wait_for_gui_mode();
    		break;
    	case BMS_CHARGING:
    		exit_charging_mode();
    		break;
    	case BMS_BALANCING:
    		exit_balancing_mode();
    		break;
    	case BMS_PRECHARGING:
    		exit_precharge_mode();
    		break;
    	case BMS_DRIVE:
    		exit_drive_mode();
    		break;
    	default:
    		// TODO: Check for race conditions.
    		// BMS_INTERNAL_FAULT or BMS_EXTERNAL_FAULT
    		// Should never reach here, ideally...
    		break;
    }
}

void wakeup_tasks() {
	switch(bms_state) {
		case BMS_WAIT_FOR_GUI:
			// NO CORRESPONDING TASKS TO WAKE
			break;
		case BMS_BALANCING:
			xTaskNotify(balancingTaskHandle, 0, eNoAction);
			break;
		case BMS_CHARGING:
			xTaskNotify(chargingTaskHandle, 0, eNoAction);
			break;
		case BMS_PRECHARGING:
			xTaskNotify(prchgTaskHandle, 0, eNoAction);
			break;
		case BMS_DRIVE:
            xTaskNotify(currLimitTaskHandle, 0, eNoAction);
            break;

		default:
			break;
	}
}

void change_baud_rate_500() {
	osMutexWait(CAN_MUTEXHandle, osWaitForever);
    HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	HAL_FDCAN_Init(&hfdcan1);
	startCAN_Tx_Rx();
	osMutexRelease(CAN_MUTEXHandle);
}

void change_baud_rate_250() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON CHARGER!
	osMutexWait(CAN_MUTEXHandle, osWaitForever);
    HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
	HAL_FDCAN_Init(&hfdcan1);
	startCAN_Tx_Rx();
	osMutexRelease(CAN_MUTEXHandle);
}

// ENTER BMS STATES!
void enter_wait_for_gui_mode() {
	bms_state = BMS_WAIT_FOR_GUI;
	// Receive commands over Serial (GUI)!
	HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rx_buffer, RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
	// NO CORRESPONDING TASKS TO WAKE!
}

void enter_precharge_mode() {
	bms_state = BMS_PRECHARGING;
	precharge_state = PRECHARGE_IDLE;
	wakeup_tasks();
}

void enter_drive_mode() {
	bms_state = BMS_DRIVE;
	// No specialized state machine for driving.
	// precharge_state should be PRECHARGE_SUCCESS.
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

	osMutexWait(SPI_MUTEXHandle, osWaitForever);
	adBms6830_write_read_config(TOTAL_IC, IC);
	osMutexRelease(SPI_MUTEXHandle);

	bms_state = BMS_BALANCING;
	balance_state = BALANCE_IDLE;
	wakeup_tasks();
}

// EXIT BMS STATES!
// Reset any specialized state machines and associated necessary variables.
void exit_wait_for_gui_mode() {
	HAL_UART_DMAStop(&hlpuart1);
}

void exit_precharge_mode() {
	precharge_state = PRECHARGE_IDLE;
}

void exit_drive_mode() {
	// NOTE: No specialized state machine associated with driving.
	// However, successful precharge is necessary for transition to driving.
	// Thus, we effectively are exiting precharge mode.
	precharge_state = PRECHARGE_IDLE;
}

void exit_charging_mode() {
	charging_state = CHG_IDLE;
	HAL_UART_DMAStop(&hlpuart1); // Prevent GUI Commands.
    // change_baud_rate_500();
	// Keep Baud Rate at 250 Kbps for debugging reasons!
	// Still want to use CAN even if we are not actively charging.
     stopPWM_Capture();
	// Control Pilot readings NOT necessary if charging is not active.
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

	osMutexWait(SPI_MUTEXHandle, osWaitForever);
	adBms6830_write_read_config(TOTAL_IC, IC);
	osMutexRelease(SPI_MUTEXHandle);
}
