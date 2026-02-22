/*
 * prchg.h
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */

#ifndef INC_PRCHG_H_
#define INC_PRCHG_H_

#include "voltage_calculations.h"
#include "gpio.h"
#include "tim.h"
#include "fdcan.h"
#include "adBms_Application.h"
#include "serialPrintResult.h"
#include <math.h>
#include <stdbool.h>

#define PRECHARGE_REQUEST_RX_ID 0x6B3
typedef union PRECHARGE_COMPLETE_DF {
	struct __attribute__((packed)) {
		uint8_t inverter_precharged;
		uint8_t reserved0;
		uint8_t reserved1;
		uint8_t reserved2;
		uint8_t reserved3;
		uint8_t reserved4;
		uint8_t reserved5;
		uint8_t reserved6;
		uint8_t reserved7;
	} data;
	uint8_t array[8];
} PRECHARGE_COMPLETE_DF;
extern PRECHARGE_COMPLETE_DF Precharge_Complete_DF;
#define PRECHARGE_COMPLETE_TX_ID 0x6B4
extern FDCAN_TxHeaderTypeDef Precharge_Complete_TxHeader;

#define PRECHARGE_VOLTAGE_DELTA 25
extern volatile float inverter_dc_volts;

typedef enum {
    PRECHARGE_IDLE,
    PRECHARGE_ACTIVE,
    PRECHARGE_COMPLETE,
	PRECHARGE_FAIL,
} PRECHARGE_STATE;
extern PRECHARGE_STATE precharge_state;

#define PRECHARGE_TIMEOUT 5000
extern bool inverter_precharged;
extern uint32_t precharge_start_time;
void prechargeStart();
void prechargeSequence();

#endif /* INC_PRCHG_H_ */
