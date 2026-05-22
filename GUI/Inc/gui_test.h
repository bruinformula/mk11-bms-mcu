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
#include "balancing.h"

extern char json_buf[4096];

void send_bal_status(uint8_t tIC, cell_asic *ic);
void send_temp_status(void);

#endif /* INC_GUI_TEST_H_ */
