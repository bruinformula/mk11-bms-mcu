/*
 * bms_state.c
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#include "bms_state.h"
#include "task.h"

bool shutdown_power = false;
volatile BMS_STATE bms_state = BMS_IDLE;

#define RX_BUF_SIZE 16
static uint8_t uart_rx_buffer[RX_BUF_SIZE];

static void notify_task(osThreadId task_handle) {
	if (task_handle == NULL) {
		return;
	}

	if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
		return;
	}

	if (__get_IPSR() != 0U) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	} else {
		xTaskNotifyGive(task_handle);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t len) {
	if (huart->Instance == LPUART1) {
		char cmd[16];
		uint16_t copy_len = (len < RX_BUF_SIZE - 1) ? len : (RX_BUF_SIZE - 1);
		memcpy(cmd, uart_rx_buffer, copy_len);
		cmd[copy_len] = '\0';

		char *newline = strchr(cmd, '\n');
		if (newline) *newline = '\0';
		char *carriage = strchr(cmd, '\r');
		if (carriage) *carriage = '\0';

		if (strcmp(cmd, "CHG_ON") == 0) {
			enter_charging_mode();
		}

		if (strcmp(cmd, "CHG_OFF") == 0) {
			// TODO
		}

		if (strcmp(cmd, "BAL_ON") == 0) {
			enter_balancing_mode();
		}

		if (strcmp(cmd, "BAL_OFF") == 0) {
			// TODO
		}

        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rx_buffer, RX_BUF_SIZE);
	}
}

void determine_startup_mode() {
	if (HAL_GPIO_ReadPin(CHARGE_SIGNAL_GPIO_Port, CHARGE_SIGNAL_Pin) == GPIO_PIN_SET) {
//		bms_state = BMS_WAIT_FOR_GUI;
//		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rx_buffer, RX_BUF_SIZE);

		enter_charging_mode();
	} else if (HAL_GPIO_ReadPin(READY_SIGNAL_GPIO_Port, READY_SIGNAL_Pin) == GPIO_PIN_SET) {
		enter_precharge_mode();
	}
}

void enter_precharge_mode() {
	bms_state = BMS_PRECHARGING;
	wakeup_tasks();
}

void enter_drive_mode() {
	bms_state = BMS_DRIVE;
	wakeup_tasks();
}

void enter_charging_mode() {
	bms_state = BMS_CHARGING;

	// Close AIRs to begin charging.
	HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_SET);

	change_baud_rate_250();
	startPWM_Capture();
	wakeup_tasks();
}

void enter_balancing_mode() {
	bms_state = BMS_BALANCING;
	// DCTO must be set non-zero.
	for (size_t i = 0; i < TOTAL_IC; ++i) {
		IC[i].tx_cfgb.dcto = 5;
	}
	adBms6830_write_config(TOTAL_IC, IC);
	wakeup_tasks();
}

void wakeup_tasks() {
	switch(bms_state) {
		case BMS_BALANCING:
			notify_task(balancingTaskHandle);
			break;
		case BMS_CHARGING:
			notify_task(chargingTaskHandle);
			break;
		case BMS_PRECHARGING:
			notify_task(prchgTaskHandle);
			break;
		case BMS_DRIVE:
            notify_task(currLimitTaskHandle);
            break;

		default:
			break;
	}
}

void change_baud_rate_250() {
	// BAUD RATE MUST BE CHANGED TO 250 KBps TO TALK TO ELCON CHARGER!
    HAL_FDCAN_DeInit(&hfdcan1);
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
	HAL_FDCAN_Init(&hfdcan1);
	startCAN_Tx_Rx();
}
