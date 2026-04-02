#ifndef BALANCING_H
#define BALANCING_H

#include <stdint.h>
#include "adBms_Application.h"
#include "adBms6830Data.h"

/* Balancing Logic Parameters */
#define BALANCE_THRESHOLD_MV       15                      // Below 15mV delta = 0% PWM
#define MAX_DELTA_MV_FOR_100_PCT   100                     // Above 100mV delta = 100% PWM

#define VOLTAGE_TO_CODES(mv)       ((mv) * 1000 / 150)
#define BALANCE_THRESHOLD_CODES    VOLTAGE_TO_CODES(BALANCE_THRESHOLD_MV)

#define STABILIZE_TIME_MS          5000    // Wait 5 seconds for chemistry to settle
#define MONITOR_TIME_MS            25000   // 25-second monitoring/read window
#define DISCHARGE_TIME_MS          30000   // 30 seconds of active balancing

void Start_Cell_Balancing(cell_asic *ic, uint8_t total_ic);

#endif /* BALANCING_H */
