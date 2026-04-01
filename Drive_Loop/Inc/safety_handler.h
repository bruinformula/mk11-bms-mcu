/*
 * safety_handler.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_SAFETY_HANDLER_H_
#define INC_SAFETY_HANDLER_H_

#define NUM_FAULT_TYPES 5

typedef enum {
	FAULT_CELL_OV = 0,
	FAULT_CELL_UV,
	FAULT_CELL_OVERTEMP,
	FAULT_CELL_UNDERTEMP,
	FAULT_OVERCURRENT,
} Fault_Types;

#endif /* INC_SAFETY_HANDLER_H_ */
