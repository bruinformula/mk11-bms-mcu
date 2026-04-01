/*
 * datalogging.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_DATALOGGING_H_
#define INC_DATALOGGING_H_

#include "fdcan.h"
#include "freertos_handles.h"
#include "state_of_charge.h"
#include "current_calculations.h"
#include "voltage_calculations.h"
#include "thermistor.h"

#define TEMP_TX_ID 0x6B0
#define VOLTAGE_TX_ID 0x6B1

typedef union TEMP_DF {
	struct __attribute__((packed)) {
		uint16_t avg_temp;
		uint16_t highest_temp;
		uint16_t lowest_temp;
		uint8_t reserved6;
		uint8_t reserved7;
	} data;
	uint8_t array[8];
} TEMP_DF;

typedef union VOLTAGE_DF {
	struct __attribute__((packed)) {
		uint16_t avg_cell_voltage;
		uint16_t lowest_cell_voltage;
		uint16_t highest_cell_voltage;
		uint16_t bms_pack_voltage;
	} data;
	uint8_t array[8];
} VOLTAGE_DF;

void configureTemp_TxMsg();
void configureVoltage_TxMsg();
void sendTemp();
void sendVoltage();

#endif /* INC_DATALOGGING_H_ */
