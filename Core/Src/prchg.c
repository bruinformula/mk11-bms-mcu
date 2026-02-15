/*
 * prchg.c
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */


#include "prchg.h"

float inverter_dc_volts;
float bms_pack_voltage;
bool inverter_precharged;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	configureFDCAN_TxMessage_STD(&FDCAN_BMS.Precharge_Complete_TxHeader, PRECHARGE_COMPLETE_TX_ID);

	if (htim->Instance == TIM1) {
		if (inverter_precharged == true) {
			FDCAN_BMS.Precharge_Complete_DF.data.inverter_precharged = 1;
			HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
		} else {
			FDCAN_BMS.Precharge_Complete_DF.data.inverter_precharged = 0;
			HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
		}

	    HAL_TIM_Base_Stop_IT(&htim1);

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
				&FDCAN_BMS.Precharge_Complete_TxHeader,
				FDCAN_BMS.Precharge_Complete_DF.array) != HAL_OK) {
			// TODO: Handle Failure
		}
	}
}

void calculatePackVoltage() {
	bms_pack_voltage = 0;
	adBms6830_read_cell_voltages(TOTAL_IC, IC);
	for (size_t i = 0; i < 16; ++i) {
		bms_pack_voltage+=(fabs(getVoltage(IC->cell.c_codes[i])));
	}
}

void prechargeSequence() {
	calculatePackVoltage();

	HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start_IT(&htim1);
	while (fabsf(bms_pack_voltage - inverter_dc_volts) > PRECHARGE_VOLTAGE_DELTA) {
		inverter_precharged = false;
	}
	inverter_precharged = true;
}
