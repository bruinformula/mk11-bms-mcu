/*
 * fault_system.h
 *
 *  Created on: Mar 9, 2026
 *      Author: ronitbarman
 */

#ifndef INC_FAULT_SYSTEM_H_
#define INC_FAULT_SYSTEM_H_

#include "adBms6830Data.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FAULT_CATEGORY_VOLTAGE = 0,
    FAULT_CATEGORY_TEMP,
    FAULT_CATEGORY_CURRENT,
    FAULT_CATEGORY_COMMUNICATION,
    FAULT_CATEGORY_HARDWARE
} FaultCategory_t;

typedef enum {
    FAULT_SEVERITY_WARNING = 0,
    FAULT_SEVERITY_ERROR,
    FAULT_SEVERITY_CRITICAL
} FaultSeverity_t;

typedef enum {
    BMS_FAULT_UV = 0,
    BMS_FAULT_OV,
    BMS_FAULT_OT,
    BMS_FAULT_UT,
    BMS_FAULT_DELTA,
    BMS_FAULT_COMM,
    BMS_FAULT_CCL_DCL,
    BMS_FAULT_HW
} BMS_FaultType_t;

typedef struct {
    uint8_t uv_active : 1;
    uint8_t ov_active : 1;
    uint8_t ot_active : 1;
    uint8_t ut_active : 1;
    uint8_t delta_active : 1;
    uint8_t comm_active : 1;
    uint8_t hw_active : 1;
    uint8_t ccl_dcl_active : 1;

    uint32_t uv_start_time_ms;
    uint32_t ov_start_time_ms;
    uint32_t last_fault_time_ms;
    uint16_t fault_count;
} BMS_CellFaultStatus_t;

typedef struct {
    BMS_CellFaultStatus_t cell_status[TOTAL_CELLS];
    uint16_t active_faults[BMS_FAULT_CATEGORY_COUNT];
    FaultSeverity_t highest_severity;
    bool system_fault_active;
    uint32_t first_fault_time_ms;
    uint32_t last_fault_clear_time_ms;
    uint16_t hw_ov_flags[TOTAL_IC];
    uint16_t hw_uv_flags[TOTAL_IC];
} BMS_FaultSystem_t;

typedef struct {
    uint8_t max_ics;
    uint8_t cells_per_ic;
} BMS_FaultConfig_t;

void BMS_FaultSystemInit(BMS_FaultSystem_t *system, const BMS_FaultConfig_t *config);
void BMS_FaultSystemReset(BMS_FaultSystem_t *system);

bool BMS_SetFault(BMS_FaultSystem_t *system,
                  uint8_t ic_index,
                  uint8_t cell_index,
                  BMS_FaultType_t fault_type,
                  FaultSeverity_t severity,
                  uint32_t now_ms);

bool BMS_ClearFault(BMS_FaultSystem_t *system,
                    uint8_t ic_index,
                    uint8_t cell_index,
                    BMS_FaultType_t fault_type,
                    uint32_t now_ms);

bool BMS_UpdateDebouncedVoltageFault(BMS_FaultSystem_t *system,
                                     uint8_t ic_index,
                                     uint8_t cell_index,
                                     BMS_FaultType_t fault_type,
                                     FaultSeverity_t severity,
                                     bool condition_active,
                                     uint32_t grace_ms,
                                     uint32_t now_ms);

uint16_t BMS_GetFaultCount(const BMS_FaultSystem_t *system, FaultCategory_t category);
bool BMS_HasActiveFaults(const BMS_FaultSystem_t *system);
FaultSeverity_t BMS_GetHighestSeverity(const BMS_FaultSystem_t *system);
const BMS_CellFaultStatus_t *BMS_GetCellFaultStatus(const BMS_FaultSystem_t *system,
                                                    uint8_t ic_index,
                                                    uint8_t cell_index);

void BMS_SetHardwareFlags(BMS_FaultSystem_t *system,
                          uint8_t ic_index,
                          uint16_t uv_flags,
                          uint16_t ov_flags);




#endif /* INC_FAULT_SYSTEM_H_ */
