/*
 * gui_test.h
 *
 *  Created on: Apr 4, 2026
 *      Author: ishanchitale
 */

#ifndef INC_GUI_TEST_H_
#define INC_GUI_TEST_H_

#include <curr_limiting.h>
#include "usart.h"
#include "current_calculations.h"
#include "thermistor.h"
#include "voltage_calculations.h"
#include "adBms_Application.h"

extern char json_buf[4096];
int build_bms_json();
void send_bms_json();

#endif /* INC_GUI_TEST_H_ */
