/*
 * fault_system.h
 *
 *  Created on: Mar 9, 2026
 *      Author: ronitbarman
 */

#ifndef INC_FAULT_SYSTEM_H_
#define INC_FAULT_SYSTEM_H_

#include "adBms6830Data.h"
#include "adBms_Application.h"
#include "serialPrintResult.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FAULT_CATEGORY_VOLTAGE = 0,
    FAULT_CATEGORY_TEMP,
    FAULT_CATEGORY_CURRENT,
    FAULT_CATEGORY_COMMUNICATION,
    FAULT_CATEGORY_HARDWARE,
	FAULT_CATEGORY_COUNT
} FaultCategory_t;

typedef enum {
    FAULT_SEVERITY_WARNING = 0,
    FAULT_SEVERITY_ERROR,
    FAULT_SEVERITY_CRITICAL
} FaultSeverity_t;

#define FAULT_TYPE_UV 1      // Under-voltage fault
#define FAULT_TYPE_OV 2      // Over-voltage fault
#define FAULT_TYPE_OT 3    // Temperature fault
#define FAULT_TYPE_UT 4
#define FAULT_TYPE_DELTA 5
#define FAULT_TYPE_COMM 6
#define FAULT_TYPE_CCL_DCL 7

#define CELL_VOLT_GRACE_MS 200

typedef struct {
    uint8_t uv_active : 1;
    uint8_t ov_active : 1;
    uint8_t ot_active : 1;
    uint8_t ut_active : 1;
    uint8_t delta_active : 1;
    uint8_t comm_active : 1;
    uint8_t hw_active : 1;
    uint8_t ccl_dcl_active : 1;

    uint32_t uv_start_time;
    uint32_t ov_start_time;
    uint32_t last_fault_time;
    uint16_t fault_count;
} CellFaultStatus_t;

typedef struct {
    CellFaultStatus_t cell_status[TOTAL_CELLS];
    uint16_t active_faults[FAULT_CATEGORY_COUNT];
    FaultSeverity_t highest_severity;
    bool system_fault_active;
    uint32_t first_fault_time;
    uint32_t last_fault_clear_time;
    uint16_t hw_ov_flags[TOTAL_IC];
    uint16_t hw_uv_flags[TOTAL_IC];
} BMS_FaultSystem_t;

typedef struct {
    uint8_t max_ics;
    uint8_t cells_per_ic;
} BMS_FaultConfig_t;

BMS_FaultSystem_t BMS_Faults;
uint8_t cell_fault = 0;
uint8_t temp_fault = 0;
uint8_t current_sensor_fault = 0;
int accy_status = 0;
float current = 0.0;
float cell_resistance = 0.00005; // 0.05 mOhm
float soc = 0.0;
float lowest_cell = 0.0;
float highest_cell = 0.0;
float avg_cell = 0.0;
float delta_cell = 0.0;

const float ov_THRESHOLD = 4.2;
const float uv_THRESHOLD = 2.5;
const float MAX_VOLTAGE_DELTA = 0.3;
const uint8_t READY_POWER = 0x01;
const uint8_t CHARGE_POWER = 0x02;


void BMS_InitFaultSystem(void);

void BMS_SetCellFault(uint8_t ic_num,
					  uint8_t cell_num,
					  uint8_t fault_type,
					  FaultSeverity_t severity);


void BMS_ClearCellFault(uint8_t ic_num,
						uint8_t cell_num,
						uint8_t fault_type);

bool BMS_HasActiveFaults(void);
uint8_t BMS_GetFaultCount(FaultCategory_t category);

void print_fault_summary(void);

int user_adBms6830_cellVoltageFaults(uint8_t tIC, cell_asic *IC);

static void BMS_DebounceUVOV(uint8_t ic, uint8_t cell_num,
                             uint8_t fault_type,
                             FaultSeverity_t severity,
                             bool condition_active,
                             uint32_t grace_ms);


FaultSeverity_t BMS_GetHighestSeverity(const BMS_FaultSystem_t *system);
const CellFaultStatus_t *GetCellFaultStatus(const BMS_FaultSystem_t *system,
                                                    uint8_t ic_index,
                                                    uint8_t cell_index);





#endif /* INC_FAULT_SYSTEM_H_ */
