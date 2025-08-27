#include "can.h"
#include "custom_functions.h"

HAL_StatusTypeDef CANTransmitMinion(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *header, uint8_t *dataArray) {
	HAL_StatusTypeDef status = HAL_ERROR;
	int attempts = 0;

	while (attempts < FDCAN_RETRY_LIMIT && status != HAL_OK) {
		status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, dataArray);
		attempts++;
	}

	return status;
}

void init_FDCAN_header(FDCAN_TxHeaderTypeDef *hdr, uint32_t id) {
	hdr->Identifier = id;
	hdr->IdType = FDCAN_STANDARD_ID;
	hdr->TxFrameType = FDCAN_DATA_FRAME;
	hdr->DataLength = FDCAN_DLC_BYTES_8;
	hdr->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	hdr->BitRateSwitch = FDCAN_BRS_OFF;
	hdr->FDFormat = FDCAN_CLASSIC_CAN;
	hdr->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	hdr->MessageMarker = 0;
}

void init_FDCAN_header_EXTENDED(FDCAN_TxHeaderTypeDef *hdr, uint32_t id) {
	hdr->Identifier = id;
	hdr->IdType = FDCAN_EXTENDED_ID;
	hdr->TxFrameType = FDCAN_DATA_FRAME;
	hdr->DataLength = FDCAN_DLC_BYTES_8;
	hdr->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	hdr->BitRateSwitch = FDCAN_BRS_OFF;
	hdr->FDFormat = FDCAN_CLASSIC_CAN;
	hdr->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	hdr->MessageMarker = 0;
}



/**
 * is charging is 1 if ur tryna charge ts accy, 0 otherwise
 *
 */
void FDCAN_BMS_Mailman(FDCAN_HandleTypeDef *hfdcan, FDCAN_BMS_CONTEXT *ctx, uint32_t now_ms, uint8_t isCharging) {
	// Message 0x6B0: current, voltage, SoC, flags
	if (now_ms - ctx->last_tx_time_6b0 >= MSG_6B0_PERIOD_MS) {
		ctx->header_6b0.Identifier = FDCAN_MSG_ID_6B0;
		ctx->header_6b0.DataLength = 8;
		CANTransmitMinion(hfdcan, &ctx->header_6b0, ctx->msg_6b0.array);
		ctx->last_tx_time_6b0 = now_ms;
	}

	// Message 0x6B1: DCL, CCL, temps
	if (now_ms - ctx->last_tx_time_6b1 >= MSG_6B1_PERIOD_MS) {
		ctx->header_6b1.Identifier = FDCAN_MSG_ID_6B1;
		CANTransmitMinion(hfdcan, &ctx->header_6b1, ctx->msg_6b1.array);
		ctx->last_tx_time_6b1 = now_ms;
	}

	// Message 0x6B2: high/low cell voltages
	if (now_ms - ctx->last_tx_time_6b2 >= MSG_6B2_PERIOD_MS) {
		ctx->header_6b2.Identifier = FDCAN_MSG_ID_6B2;
		CANTransmitMinion(hfdcan, &ctx->header_6b2, ctx->msg_6b2.array);
		ctx->last_tx_time_6b2 = now_ms;
	}

	if (isCharging) {
		while ((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0)
				{
				    // TX FIFO queue is full, wait
				}
		// Message 0x6B0: current, voltage, SoC, flags
		if (now_ms - ctx->CAN_CHGCONTEXT.last_tx_time_1806e7f4 >= MSG_CHARGER_PERIOD_MS) {
			ctx->CAN_CHGCONTEXT.header_1806E7F4.Identifier = 0x1806e7f4;
			CANTransmitMinion(hfdcan, &ctx->CAN_CHGCONTEXT.header_1806E7F4, ctx->CAN_CHGCONTEXT.chgmsg_1806e7f4.array);
			ctx->CAN_CHGCONTEXT.last_tx_time_1806e7f4 = now_ms;
		}
		while ((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0)
						{
						    // TX FIFO queue is full, wait
						}
		// Message 0x6B1: DCL, CCL, temps
		if (now_ms - ctx->CAN_CHGCONTEXT.last_tx_time_1806e5f4 >= MSG_CHARGER_PERIOD_MS) {
			ctx->CAN_CHGCONTEXT.header_1806E5F4.Identifier = 0x1806e5f4;

			CANTransmitMinion(hfdcan, &ctx->CAN_CHGCONTEXT.header_1806E5F4, ctx->CAN_CHGCONTEXT.chgmsg_1806e5f4.array);
			ctx->CAN_CHGCONTEXT.last_tx_time_1806e5f4 = now_ms;
		}
		while ((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0)
						{
						    // TX FIFO queue is full, wait
						}
		// Message 0x6B2: high/low cell voltages
		if (now_ms - ctx->CAN_CHGCONTEXT.last_tx_time_1806e9f4 >= MSG_CHARGER_PERIOD_MS) {
			ctx->CAN_CHGCONTEXT.header_1806E9F4.Identifier = 0x1806e9f4;

			CANTransmitMinion(hfdcan, &ctx->CAN_CHGCONTEXT.header_1806E9F4, ctx->CAN_CHGCONTEXT.chgmsg_1806e9f4.array);
			ctx->CAN_CHGCONTEXT.last_tx_time_1806e9f4 = now_ms;
		}
		while ((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0)
						{
						    // TX FIFO queue is full, wait
						}
		// Message 0x6B2: high/low cell voltages
		if (now_ms - ctx->CAN_CHGCONTEXT.last_tx_time_18ff50e5 >= MSG_CHARGER_PERIOD_MS) {
			ctx->CAN_CHGCONTEXT.header_18FF50E5.Identifier = 0x18ff50e5;

			CANTransmitMinion(hfdcan, &ctx->CAN_CHGCONTEXT.header_18FF50E5, ctx->CAN_CHGCONTEXT.chgmsg_18ff50e5.array);
			ctx->CAN_CHGCONTEXT.last_tx_time_18ff50e5 = now_ms;
		}

	}
}
