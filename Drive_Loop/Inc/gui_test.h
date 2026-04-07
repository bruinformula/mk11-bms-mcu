/*
 * gui_test.h
 *
 *  Created on: Apr 4, 2026
 *      Author: ishanchitale
 */

#ifndef INC_GUI_TEST_H_
#define INC_GUI_TEST_H_

#include "current_calculations.h"
#include "thermistor.h"
#include "voltage_calculations.h"
#include "adBms_Application.h"

#include "bms_state.h"
#include "state_of_charge.h"
#include "currLimiting.h"

#define JSON_BUF_SIZE 8192
#define ICS_PER_SEGMENT 2
/* Ceiling division so 1 IC still = 1 segment */
#define NUM_SEGMENTS ((TOTAL_IC + ICS_PER_SEGMENT - 1) / ICS_PER_SEGMENT)

extern char json_buf[JSON_BUF_SIZE];
int build_bms_json(void);

#endif /* INC_GUI_TEST_H_ */
