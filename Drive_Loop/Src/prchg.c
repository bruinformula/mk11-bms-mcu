/*
 * prchg.c
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */


#include "prchg.h"

volatile float inverter_dc_volts;
volatile bool inverter_precharged = false;
static uint32_t precharge_start_time = 0;
static FDCAN_TxHeaderTypeDef Precharge_Complete_TxHeader;
static PRECHARGE_COMPLETE_DF Precharge_Complete_DF;
volatile static PRECHARGE_STATE precharge_state = PRECHARGE_IDLE;

void configurePrchgTxMsg() {
	configureFDCAN_TxMessage_STD(&Precharge_Complete_TxHeader, PRECHARGE_COMPLETE_TX_ID);
}

void prechargeStart() { // Triggered by CAN Request from VCU; check fdcan.c.
	if (precharge_state == PRECHARGE_IDLE || precharge_state == PRECHARGE_FAIL) { // Able to re-attempt precharge!
		HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_SET);
		precharge_start_time = HAL_GetTick();
	    precharge_state = PRECHARGE_ACTIVE;
	}
}

void precharge_loop() {
	switch (precharge_state) {
		case PRECHARGE_IDLE:
			break;

		case PRECHARGE_ACTIVE:
            if (HAL_GetTick() - precharge_start_time < PRECHARGE_TIMEOUT_MS) {
                break;
            }

            float delta = fabsf(voltage_context.estimated_pack_voltage - inverter_dc_volts);

            if (delta <= PRECHARGE_VOLTAGE_DELTA) {
        		inverter_precharged = true;
        		HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
        		precharge_state = PRECHARGE_SUCCESS;
        		Precharge_Complete_DF.data.inverter_precharged = 1;

        		osMutexWait(CAN_MUTEXHandle, osWaitForever);
        		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Precharge_Complete_TxHeader, Precharge_Complete_DF.array);
        		osMutexRelease(CAN_MUTEXHandle);

    			bms_state = BMS_DRIVE;
    			wakeup_tasks();
            } else {
            	inverter_precharged = false;
            	HAL_GPIO_WritePin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_RESET);
            	precharge_state = PRECHARGE_FAIL;
            	Precharge_Complete_DF.data.inverter_precharged = 0;

        		osMutexWait(CAN_MUTEXHandle, osWaitForever);
            	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Precharge_Complete_TxHeader, Precharge_Complete_DF.array);
        		osMutexRelease(CAN_MUTEXHandle);
            }
            break;

		case PRECHARGE_SUCCESS:
			break;

		case PRECHARGE_FAIL:
			break;

	}
}
