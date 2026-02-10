#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32g4xx_hal.h"
#include "dataframes.h"

/* FDCAN Message IDs */
#define FDCAN_MSG_ID_6B0  0x6B0
#define FDCAN_MSG_ID_6B1  0x6B1
#define FDCAN_MSG_ID_6B2  0x6B2

/* Retry attempts for FDCAN transmit */
#define FDCAN_RETRY_LIMIT  3

/* Transmission periods in milliseconds */
#define MSG_6B0_PERIOD_MS   8     // High-frequency: current, voltage
#define MSG_6B1_PERIOD_MS   104   // Lower-frequency: limits, temps
#define MSG_6B2_PERIOD_MS   8     // High-frequency: cell voltage extremes
#define MSG_CHARGER_PERIOD_MS   100     // charger status messages

/** CAN transmission context for specifically charging **/
typedef struct FDCAN_CHARGER_CONTEXT{

	FDCAN_TxHeaderTypeDef header_1806E7F4;
	FDCAN_TxHeaderTypeDef header_1806E5F4;
	FDCAN_TxHeaderTypeDef header_1806E9F4;
	FDCAN_TxHeaderTypeDef header_18FF50E5;

	msg_1806e7f4 chgmsg_1806e7f4;
	msg_1806e5f4 chgmsg_1806e5f4;
	msg_1806e9f4 chgmsg_1806e9f4;
	msg_18ff50e5 chgmsg_18ff50e5;

	uint32_t last_tx_time_1806e7f4;
	uint32_t last_tx_time_1806e5f4;
	uint32_t last_tx_time_1806e9f4;
	uint32_t last_tx_time_18ff50e5;

} FDCAN_CHARGER_CONTEXT;

/* CAN Transmission Context */
typedef struct {
	FDCAN_TxHeaderTypeDef header_6b0;
	FDCAN_TxHeaderTypeDef header_6b1;
	FDCAN_TxHeaderTypeDef header_6b2;
	FDCAN_TxHeaderTypeDef header_6b3;

	CAN1_DATAFRAME msg_6b0;
	CAN2_DATAFRAME msg_6b1;
	CAN3_DATAFRAME msg_6b2;
	CAN4_DATAFRAME msg_6b3;

	uint32_t last_tx_time_6b0;
	uint32_t last_tx_time_6b1;
	uint32_t last_tx_time_6b2;
	uint32_t last_tx_time_6b3;

	FDCAN_CHARGER_CONTEXT CAN_CHGCONTEXT;

} FDCAN_BMS_CONTEXT;

/* Public method declarations */
void FDCAN_BMS_Mailman(FDCAN_HandleTypeDef *hfdcan, FDCAN_BMS_CONTEXT *ctx, uint32_t now_ms, uint8_t isCharging);
void init_FDCAN_header(FDCAN_TxHeaderTypeDef *hdr, uint32_t id);
void init_FDCAN_header_EXTENDED(FDCAN_TxHeaderTypeDef *hdr, uint32_t id);

#endif /* INC_CAN_H_ */
