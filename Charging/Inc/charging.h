/*
 * charging.h
 *
 *  Created on: Feb 17, 2026
 *      Author: ishanchitale
 */

#ifndef INC_CHARGING_H_
#define INC_CHARGING_H_

#include "j_plug.h"
#include "elcon_charger.h"
#include "fdcan.h"
#include "voltage_calculations.h"
#include "thermistor.h"

#define CHARGER_VOLTAGE 420
#define MAX_TEMPERATURE_CHARGING_THRESHOLD 60.0
#define MAX_CELL_VOLTAGE_CHARGING_THRESHOLD 4.0

void change_baud_rate();
void charging_sequence_startup();
void charging_sequence();

#endif /* INC_CHARGING_H_ */
