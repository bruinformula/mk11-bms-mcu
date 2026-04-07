/*
 * prchg.c
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */


#include "prchg.h"

volatile float inverter_dc_volts;
volatile bool inverter_precharged = false;
static FDCAN_TxHeaderTypeDef Precharge_Complete_TxHeader;
static PRECHARGE_COMPLETE_DF Precharge_Complete_DF;
volatile static PRECHARGE_STATE precharge_state = PRECHARGE_IDLE;

//static void delay_10ms(void) {
//    uint32_t start = DWT->CYCCNT;
//    uint32_t ticks = SystemCoreClock / 100; // 10 ms = 1/100 sec
//
//    while ((DWT->CYCCNT - start) < ticks);
//}

void prechargeStart() {
	HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_SET);
//	delay_10ms();
	HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_SET);
//	delay_10ms();
	configureFDCAN_TxMessage_STD(&Precharge_Complete_TxHeader, PRECHARGE_COMPLETE_TX_ID);
    precharge_state = PRECHARGE_ACTIVE;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim1);
}

// NOTE: This function is called within the auto-generated callback for TIM Period Elapsed Callback
// This is because FreeRTOS generates it in main.c
void prechargeCheck() {
	float delta = fabsf(bms_pack_voltage - inverter_dc_volts);
	if (delta <= PRECHARGE_VOLTAGE_DELTA) {
		inverter_precharged = true;
		HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
//		delay_10ms();
		HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
//		delay_10ms();
		precharge_state = PRECHARGE_COMPLETE;
		Precharge_Complete_DF.data.inverter_precharged = 1;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Precharge_Complete_TxHeader, Precharge_Complete_DF.array);
		enterDriveMode();
	} else {
		inverter_precharged = false;
		HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_RESET);
//		delay_10ms();
		HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
//		delay_10ms();
		precharge_state = PRECHARGE_FAIL;
		Precharge_Complete_DF.data.inverter_precharged = 0;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Precharge_Complete_TxHeader, Precharge_Complete_DF.array);
	}
	HAL_TIM_Base_Stop_IT(&htim1);
}
