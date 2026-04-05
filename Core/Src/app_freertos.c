/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "voltage_calculations.h"
#include "thermistor.h"
#include "currLimiting.h"
#include "datalogging.h"
#include "gui_test.h"
#include "usart.h"
#include "adBms6830GenericType.h"
#include "bms_state.h"
#include "balancing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t uart_rx_byte = 0;
/* USER CODE END Variables */
osThreadId voltageTaskHandle;
osThreadId tempTaskHandle;
osThreadId currLimitTaskHandle;
osThreadId dataloggingTaskHandle;
osThreadId balancingTaskHandle;
osMutexId SPI_MUTEXHandle;
osMutexId CAN_MUTEXHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void voltageFunction(void const * argument);
void tempFunction(void const * argument);
void currLimitFunction(void const * argument);
void dataloggingFunction(void const * argument);
void balancingFunction(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of SPI_MUTEX */
  osMutexDef(SPI_MUTEX);
  SPI_MUTEXHandle = osMutexCreate(osMutex(SPI_MUTEX));

  /* definition and creation of CAN_MUTEX */
  osMutexDef(CAN_MUTEX);
  CAN_MUTEXHandle = osMutexCreate(osMutex(CAN_MUTEX));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of voltageTask */
  osThreadDef(voltageTask, voltageFunction, osPriorityNormal, 0, 512);
  voltageTaskHandle = osThreadCreate(osThread(voltageTask), NULL);

  /* definition and creation of tempTask */
  osThreadDef(tempTask, tempFunction, osPriorityNormal, 0, 512);
  tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);

  /* definition and creation of currLimitTask */
  osThreadDef(currLimitTask, currLimitFunction, osPriorityBelowNormal, 0, 256);
  currLimitTaskHandle = osThreadCreate(osThread(currLimitTask), NULL);

  /* definition and creation of dataloggingTask */
  osThreadDef(dataloggingTask, dataloggingFunction, osPriorityLow, 0, 512);
  dataloggingTaskHandle = osThreadCreate(osThread(dataloggingTask), NULL);

  /* definition and creation of balancingTask */
  osThreadDef(balancingTask, balancingFunction, osPriorityAboveNormal, 0, 512);
  balancingTaskHandle = osThreadCreate(osThread(balancingTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_voltageFunction */
/**
  * @brief  Function implementing the voltageTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_voltageFunction */
void voltageFunction(void const * argument)
{
  /* USER CODE BEGIN voltageFunction */
  /* Infinite loop */
  for(;;)
  {
	if (bms_state == BMS_BALANCING) {
		osDelay(500);
		continue;
	}
	osMutexWait(SPI_MUTEXHandle, osWaitForever);
	computeAllVoltages(TOTAL_IC, IC);
	osMutexRelease(SPI_MUTEXHandle);
	osDelay(2000);
  }
  /* USER CODE END voltageFunction */
}

/* USER CODE BEGIN Header_tempFunction */
/**
* @brief Function implementing the tempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tempFunction */
void tempFunction(void const * argument)
{
  /* USER CODE BEGIN tempFunction */
  /* Infinite loop */
  for(;;)
  {
	if (bms_state == BMS_BALANCING) {
		osDelay(500);
		continue;
	}
	osMutexWait(SPI_MUTEXHandle, osWaitForever);
	computeAllTemps(TOTAL_IC, IC);
	osMutexRelease(SPI_MUTEXHandle);
    osDelay(2000);
  }
  /* USER CODE END tempFunction */
}

/* USER CODE BEGIN Header_currLimitFunction */
/**
* @brief Function implementing the currLimitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_currLimitFunction */
void currLimitFunction(void const * argument)
{
  /* USER CODE BEGIN currLimitFunction */
  /* Infinite loop */
  for(;;)
  {
	osMutexWait(CAN_MUTEXHandle, osWaitForever);
	sendDCL_CCL();
	osMutexRelease(CAN_MUTEXHandle);
    osDelay(500);
  }
  /* USER CODE END currLimitFunction */
}

/* USER CODE BEGIN Header_dataloggingFunction */
/**
* @brief Function implementing the dataloggingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dataloggingFunction */
void dataloggingFunction(void const * argument)
{
  /* USER CODE BEGIN dataloggingFunction */
  /* Infinite loop */
  for(;;)
  {
	if (HAL_UART_Receive(&hlpuart1, &uart_rx_byte, 1, 10) == HAL_OK) {
		if (uart_rx_byte == 'B') bms_state = BMS_BALANCING;
		else if (uart_rx_byte == 'S') bms_state = BMS_IDLE;
	}

	taskENTER_CRITICAL();
	int len = build_bms_json();
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)json_buf, len, HAL_MAX_DELAY);
	taskEXIT_CRITICAL();
    osDelay(2000);
  }
  /* USER CODE END dataloggingFunction */
}

/* USER CODE BEGIN Header_balancingFunction */
/**
* @brief Function implementing the balancingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_balancingFunction */
void balancingFunction(void const * argument)
{
  /* USER CODE BEGIN balancingFunction */
  osDelay(5000);
  for(;;)
  {
	if (bms_state == BMS_BALANCING) {
		osMutexWait(SPI_MUTEXHandle, osWaitForever);
		balancingLoop(TOTAL_IC, IC);
		osMutexRelease(SPI_MUTEXHandle);
	}
	osDelay(2000);
  }
  /* USER CODE END balancingFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

