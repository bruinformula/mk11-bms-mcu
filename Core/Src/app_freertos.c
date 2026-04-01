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
#include "freertos_handles.h"
#include "datalogging.h"
#include "currLimiting.h"
#include "voltage_calculations.h"
#include "thermistor.h"
#include "adBms_Application.h"
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

/* USER CODE END Variables */
osThreadId safetyTaskHandle;
osThreadId voltageTaskHandle;
osThreadId tempTaskHandle;
osThreadId currLimitTaskHandle;
osThreadId socTaskHandle;
osThreadId daqTaskHandle;
osMutexId CAN_MutexHandle;
osMutexId SPI_MUTEXHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void safetyTaskFunction(void const * argument);
void voltageTaskFunction(void const * argument);
void tempTaskFunction(void const * argument);
void currLimitTaskFunction(void const * argument);
void socTaskFunction(void const * argument);
void daqTaskFunction(void const * argument);

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
  /* definition and creation of CAN_Mutex */
  osMutexDef(CAN_Mutex);
  CAN_MutexHandle = osMutexCreate(osMutex(CAN_Mutex));

  /* definition and creation of SPI_MUTEX */
  osMutexDef(SPI_MUTEX);
  SPI_MUTEXHandle = osMutexCreate(osMutex(SPI_MUTEX));

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
  /* definition and creation of safetyTask */
  osThreadDef(safetyTask, safetyTaskFunction, osPriorityHigh, 0, 64);
  safetyTaskHandle = osThreadCreate(osThread(safetyTask), NULL);

  /* definition and creation of voltageTask */
  osThreadDef(voltageTask, voltageTaskFunction, osPriorityAboveNormal, 0, 64);
  voltageTaskHandle = osThreadCreate(osThread(voltageTask), NULL);

  /* definition and creation of tempTask */
  osThreadDef(tempTask, tempTaskFunction, osPriorityAboveNormal, 0, 64);
  tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);

  /* definition and creation of currLimitTask */
  osThreadDef(currLimitTask, currLimitTaskFunction, osPriorityNormal, 0, 64);
  currLimitTaskHandle = osThreadCreate(osThread(currLimitTask), NULL);

  /* definition and creation of socTask */
  osThreadDef(socTask, socTaskFunction, osPriorityBelowNormal, 0, 64);
  socTaskHandle = osThreadCreate(osThread(socTask), NULL);

  /* definition and creation of daqTask */
  osThreadDef(daqTask, daqTaskFunction, osPriorityLow, 0, 64);
  daqTaskHandle = osThreadCreate(osThread(daqTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_safetyTaskFunction */
/**
  * @brief  Function implementing the safetyTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_safetyTaskFunction */
void safetyTaskFunction(void const * argument)
{
  /* USER CODE BEGIN safetyTaskFunction */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END safetyTaskFunction */
}

/* USER CODE BEGIN Header_voltageTaskFunction */
/**
* @brief Function implementing the voltageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_voltageTaskFunction */
void voltageTaskFunction(void const * argument)
{
  /* USER CODE BEGIN voltageTaskFunction */
  TickType_t lastWakeTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
    computeAllVoltages(TOTAL_IC, IC);
    osDelayUntil(&lastWakeTime, 200);
  }
  /* USER CODE END voltageTaskFunction */
}

/* USER CODE BEGIN Header_tempTaskFunction */
/**
* @brief Function implementing the tempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tempTaskFunction */
void tempTaskFunction(void const * argument)
{
  /* USER CODE BEGIN tempTaskFunction */
  TickType_t lastWakeTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	computeAllTemps(TOTAL_IC, IC);
    osDelayUntil(&lastWakeTime, 200);
  }
  /* USER CODE END tempTaskFunction */
}

/* USER CODE BEGIN Header_currLimitTaskFunction */
/**
* @brief Function implementing the currLimitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_currLimitTaskFunction */
void currLimitTaskFunction(void const * argument)
{
  /* USER CODE BEGIN currLimitTaskFunction */
  TickType_t lastWakeTime = osKernelSysTick();

  /* Infinite loop */
  for(;;)
  {
	sendCCL_DCL();
    osDelayUntil(&lastWakeTime, 50);
  }
  /* USER CODE END currLimitTaskFunction */
}

/* USER CODE BEGIN Header_socTaskFunction */
/**
* @brief Function implementing the socTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_socTaskFunction */
void socTaskFunction(void const * argument)
{
  /* USER CODE BEGIN socTaskFunction */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END socTaskFunction */
}

/* USER CODE BEGIN Header_daqTaskFunction */
/**
* @brief Function implementing the daqTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_daqTaskFunction */
void daqTaskFunction(void const * argument)
{
  /* USER CODE BEGIN daqTaskFunction */
  TickType_t lastWakeTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	sendTemp();
	sendVoltage();
    osDelayUntil(&lastWakeTime, 1000);
  }
  /* USER CODE END daqTaskFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

