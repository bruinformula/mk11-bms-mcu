/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "prchg.h"
#include "elcon_charger.h"
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 20;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
FDCAN_BMS_CONTEXT FDCAN_BMS;

void configureFDCAN_TxMessage_STD(FDCAN_TxHeaderTypeDef* tx_msg, uint32_t std_id) {
	tx_msg->Identifier = std_id;
	tx_msg->IdType = FDCAN_STANDARD_ID;
	tx_msg->TxFrameType = FDCAN_DATA_FRAME;
	tx_msg->DataLength = FDCAN_DLC_BYTES_8;
	tx_msg->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_msg->BitRateSwitch = FDCAN_BRS_OFF;
	tx_msg->FDFormat = FDCAN_CLASSIC_CAN;
	tx_msg->TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	tx_msg->MessageMarker = 0;
}

void configureFDCAN_TxMessage_EXTD(FDCAN_TxHeaderTypeDef* tx_msg, uint32_t extd_id) {
	tx_msg->Identifier = extd_id;
	tx_msg->IdType = FDCAN_EXTENDED_ID;
	tx_msg->TxFrameType = FDCAN_DATA_FRAME;
	tx_msg->DataLength = FDCAN_DLC_BYTES_8;
	tx_msg->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_msg->BitRateSwitch = FDCAN_BRS_OFF;
	tx_msg->FDFormat = FDCAN_CLASSIC_CAN;
	tx_msg->TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	tx_msg->MessageMarker = 0;
}

FDCAN_RxHeaderTypeDef BMS_RxHeader;
uint8_t BMS_RxData[8];
uint32_t fdcan_rx_count;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &BMS_RxHeader, BMS_RxData) != HAL_OK) {
			// TODO
		}

		fdcan_rx_count++;
		BMS_CAN_RxHandler();
	}
}

void BMS_CAN_RxHandler() {
	uint32_t msg_id = BMS_RxHeader.Identifier;
	switch (msg_id) {

	case ELCON_CHARGER_RX_ID:
		parseChargerBroadcast();
		break;

	case PRECHARGE_REQUEST_RX_ID:
		prechargeStart();
		prechargeSequence();
		break;

	case INVERTER_VOLTAGE_RX_ID:
		int16_t inverter_dc_volts_raw = (int16_t) ((BMS_RxData[1] << 8) | BMS_RxData[0]);
		inverter_dc_volts = inverter_dc_volts_raw*0.1;
		break;
	}
}
/* USER CODE END 1 */
