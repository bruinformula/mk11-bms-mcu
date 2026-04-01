/*
 * freertos_handles.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_FREERTOS_HANDLES_H_
#define INC_FREERTOS_HANDLES_H_

#include "cmsis_os.h"

extern osThreadId safetyTaskHandle;
extern osThreadId voltageTaskHandle;
extern osThreadId tempTaskHandle;
extern osThreadId currLimitTaskHandle;
extern osThreadId socTaskHandle;
extern osThreadId daqTaskHandle;
extern osMutexId CAN_MutexHandle;
extern osMutexId SPI_MUTEXHandle;

#endif /* INC_FREERTOS_HANDLES_H_ */
