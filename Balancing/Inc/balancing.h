/*
 * balancing.h
 *
 *  Created on: Mar 9, 2026
 *      Author: ronitbarman
 */

#ifndef INC_BALANCING_H_
#define INC_BALANCING_H_

#include <stdint.h>
#include <voltage_calculations.h>

void balanceCells(uint8_t tIC, cell_asic *ic, PWM_DUTY duty_cycle);
void stopBalancing(uint8_t tIC, cell_asic *ic);


#endif /* INC_BALANCING_H_ */
