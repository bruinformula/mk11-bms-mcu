/*
 * prchg.h
 *
 *  Created on: Feb 11, 2026
 *      Author: ishanchitale
 */

#ifndef INC_PRCHG_H_
#define INC_PRCHG_H_

#include "gpio.h"
#include "tim.h"
#include "fdcan.h"
#include "adBms_Application.h"
#include "serialPrintResult.h"
#include <math.h>
#include <stdbool.h>

#define PRECHARGE_VOLTAGE_DELTA 25
extern float inverter_dc_volts;
extern float pack_voltage;
extern bool inverter_precharged;

void calculatePackVoltage();
void prechargeSequence();

#endif /* INC_PRCHG_H_ */
