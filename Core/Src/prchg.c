/*
 * prchg.c
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */


#include "prchg.h"

volatile float inverter_dc_volts;
float bms_pack_voltage;
bool inverter_precharged;
uint32_t precharge_start_time;
FDCAN_TxHeaderTypeDef Precharge_Complete_TxHeader;
PRECHARGE_COMPLETE_DF Precharge_Complete_DF;
PRECHARGE_STATE precharge_state = PRECHARGE_IDLE;

void calculatePackVoltage() {
	bms_pack_voltage = 0;
	adBms6830_start_adc_cell_voltage_measurment(TOTAL_IC);
	adBms6830_read_cell_voltages(TOTAL_IC, IC);
	for (size_t i = 0; i < 16; ++i) {
		bms_pack_voltage+=(fabs(getVoltage(IC->cell.c_codes[i])));
	}
}

void prechargeStart() {
	HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	inverter_precharged = false;
	configureFDCAN_TxMessage_STD(&Precharge_Complete_TxHeader, PRECHARGE_COMPLETE_TX_ID);

    calculatePackVoltage();
    precharge_start_time = HAL_GetTick();
    precharge_state = PRECHARGE_ACTIVE;
}

void prechargeSequence() {
	while(1) {
        float delta = fabsf(bms_pack_voltage - inverter_dc_volts);

        if (delta <= PRECHARGE_VOLTAGE_DELTA) {
        	inverter_precharged = true;
            HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            precharge_state = PRECHARGE_COMPLETE;
            Precharge_Complete_DF.data.inverter_precharged = 1;
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
            				&Precharge_Complete_TxHeader,
            				Precharge_Complete_DF.array);
            break;
		} else if (HAL_GetTick() - precharge_start_time >= PRECHARGE_TIMEOUT) {
            inverter_precharged = false;
            HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            precharge_state = PRECHARGE_FAIL;
            Precharge_Complete_DF.data.inverter_precharged = 0;
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
            				&Precharge_Complete_TxHeader,
            				Precharge_Complete_DF.array);
            break;
		}
	}
}

