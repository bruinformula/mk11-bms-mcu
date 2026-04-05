/*
 * currLimiting.h
 *
 *  Created on: Mar 31, 2026
 *      Author: ishanchitale
 */

#ifndef INC_CURRLIMITING_H_
#define INC_CURRLIMITING_H_

#include "thermistor.h"
#include "fdcan.h"

#define CCL_DCL_TX_ID 0x202
#define CCL_CURVE_POINTS 13
#define DCL_CURVE_POINTS 13

typedef union CCL_DCL_DF {
	struct __attribute__((packed)) {
		uint16_t pack_dcl;
		uint16_t pack_ccl;
		uint8_t reserved4;
		uint8_t reserved5;
		uint8_t reserved6;
		uint8_t reserved7;
	} data;
	uint8_t array[8];
} CCL_DCL_DF;

typedef struct CurvePoint {
	float temp;
	float current;
} CurvePoint;

extern float ccl;
extern float dcl;

void calculateCCL();
void calculateDCL();
void configureCCL_DCL_TxMsg();
void sendDCL_CCL();

#endif /* INC_CURRLIMITING_H_ */
