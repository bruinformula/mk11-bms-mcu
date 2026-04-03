#include "fault_system.h"



/**
 * @brief Initialize fault system
 */
void BMS_InitFaultSystem(void) {
    memset(&BMS_Faults, 0, sizeof(BMS_FaultSystem_t));
    BMS_Faults.system_fault_active = false;
    BMS_Faults.highest_severity = FAULT_SEVERITY_WARNING;
}

void BMS_SetCellFault(uint8_t ic_num, uint8_t cell_num, uint8_t fault_type,
                       FaultSeverity_t severity) {

	// UNSURE FIX 2
	// should probably change this to make sense with BMS_SetCellFault?
    if (ic_num >= 10 || cell_num >= 16) return;

    uint8_t cell_idx = (ic_num * 10) + cell_num;
    CellFaultStatus_t *cell = &BMS_Faults.cell_status[cell_idx];

    uint32_t now = HAL_GetTick();
    bool new_fault = false;

    switch (fault_type) {
        case FAULT_TYPE_UV:
            if (!cell->uv_active) {
                new_fault = true;
                cell->uv_active = 1;
                BMS_Faults.active_faults[FAULT_CATEGORY_VOLTAGE]++;
            }
            break;

        case FAULT_TYPE_OV:
            if (!cell->ov_active) {
                new_fault = true;
                cell->ov_active = 1;
                BMS_Faults.active_faults[FAULT_CATEGORY_VOLTAGE]++;
            }
            break;

        case FAULT_TYPE_OT:
            if (!cell->ot_active) {
                new_fault = true;
                cell->ot_active = 1;
                BMS_Faults.active_faults[FAULT_CATEGORY_TEMP]++;
            }
            break;

        case FAULT_TYPE_UT:
        	if (!cell->ut_active) {
        		new_fault = true;
        		cell->ut_active = 1;
        		BMS_Faults.active_faults[FAULT_CATEGORY_TEMP]++;
        	}
        	break;

		// UNSURE FIX 1
		// what is this and why does it use ov_active?
        case FAULT_TYPE_DELTA:
        	if (!cell->ov_active) {
        		new_fault = true;
        		cell->ov_active = 1;
        		BMS_Faults.active_faults[FAULT_CATEGORY_VOLTAGE]++;
        	}
        	break;

        case FAULT_TYPE_COMM:
        	if (!cell->comm_active) {
        		new_fault = true;
        		cell->comm_active = 1;
        		BMS_Faults.active_faults[FAULT_CATEGORY_COMMUNICATION]++;
        	}
        	break;

        case FAULT_TYPE_CCL_DCL:
        	if (!cell->ccl_dcl_active) {
        		new_fault = true;
        		cell->ccl_dcl_active = 1;
        		// CCL/DCL is a part of temps
        		BMS_Faults.active_faults[FAULT_CATEGORY_TEMP]++;
        	}

    }

    if (new_fault) {
        cell->last_fault_time = now;
        cell->fault_count++;

        if (BMS_Faults.first_fault_time == 0) {
            BMS_Faults.first_fault_time = now;
        }

        if (severity > BMS_Faults.highest_severity) {
        	BMS_Faults.highest_severity = severity;
        }

        BMS_Faults.system_fault_active = true;

        #if PRINT_ON
        printf("FAULT: IC%d Cell%d Type:%d Sev:%d\n",
        		ic_num, cell_num, fault_type, severity);
        #endif
    }
}

void BMS_ClearCellFault(uint8_t ic_num, uint8_t cell_num, uint8_t fault_type) {

	// UNSURE FIX 2
	// should probably change this?
    if (ic_num >= TOTAL_IC || cell_num >= CELLS_PER_IC) return;

    uint8_t cell_idx = (ic_num * CELLS_PER_IC) + cell_num;
    CellFaultStatus_t *cell = &BMS_Faults.cell_status[cell_idx];

    bool fault_cleared = false;

    switch (fault_type) {
        case FAULT_TYPE_UV:
            if (cell->uv_active) {
                cell->uv_active = 0;
                BMS_Faults.active_faults[FAULT_CATEGORY_VOLTAGE]--;
                fault_cleared = true;
            }
            break;

        case FAULT_TYPE_OV:
            if (cell->ov_active) {
                cell->ov_active = 0;
                BMS_Faults.active_faults[FAULT_CATEGORY_VOLTAGE]--;
                fault_cleared = true;
            }
            break;

        case FAULT_TYPE_OT:
            if (cell->ot_active) {
                cell->ot_active = 0;
                BMS_Faults.active_faults[FAULT_CATEGORY_TEMP]--;
                fault_cleared = true;
            }
            break;

        case FAULT_TYPE_UT:
        	if (cell->ut_active) {
        		cell->ut_active = 0;
        		BMS_Faults.active_faults[FAULT_CATEGORY_TEMP]--;
        		fault_cleared = true;
        	}
        	break;

        case FAULT_TYPE_COMM:
        	if (cell->comm_active) {
        		cell->comm_active = 0;
        		BMS_Faults.active_faults[FAULT_CATEGORY_COMMUNICATION]--;
        		fault_cleared = true;
        	}
        	break;
	    case FAULT_TYPE_CCL_DCL:
        	if (cell->ccl_dcl_active) {
        		cell->ccl_dcl_active = 0;
        		BMS_Faults.active_faults[FAULT_CATEGORY_TEMP]--;
				fault_cleared = true;
        	}
			break;
    }

    if (fault_cleared) {
        BMS_Faults.last_fault_clear_time = HAL_GetTick();

        // Check if all faults cleared
        uint8_t total_faults = 0;
        for (int i = 0; i < FAULT_CATEGORY_COUNT; i++) {
            total_faults += BMS_Faults.active_faults[i];
        }

        if (total_faults == 0) {
            BMS_Faults.system_fault_active = false;
            BMS_Faults.highest_severity = FAULT_SEVERITY_WARNING;
        }
    }
}

/**
 * @brief Get fault count for specific category
 */
uint8_t BMS_GetFaultCount(FaultCategory_t category) {
    if (category < FAULT_CATEGORY_COUNT) {
        return BMS_Faults.active_faults[category];
    }
    return 0;
}

/**
 * @brief Get system fault status for precharge integration
 */
bool BMS_HasActiveFaults(void) {
    return BMS_Faults.system_fault_active;
}

static void BMS_DebounceUVOV(uint8_t ic, uint8_t cell_num,
                             uint8_t fault_type,
                             FaultSeverity_t severity,
                             bool condition_active,
                             uint32_t grace_ms)
{
    // Bounds check (match your BMS_SetCellFault limits)
    if (ic >= TOTAL_IC || cell_num >= CELLS_PER_IC) return;

    // Indexing must match BMS_SetCellFault (ic*10 + cell)
    uint8_t cell_idx = (ic * 10) + cell_num;
    CellFaultStatus_t *cell = &BMS_Faults.cell_status[cell_idx];

    uint32_t now = HAL_GetTick();

    if (fault_type == FAULT_TYPE_UV) {

        if (condition_active) {
            if (cell->uv_start_time == 0) {
                cell->uv_start_time = now;  // start timing
            }

            if ((now - cell->uv_start_time) >= grace_ms) {
                if (!cell->uv_active) {
                    BMS_SetCellFault(ic, cell_num, FAULT_TYPE_UV, severity);
                }
            }
        } else {
            cell->uv_start_time = 0; // reset timing
            if (cell->uv_active) {
                BMS_ClearCellFault(ic, cell_num, FAULT_TYPE_UV);
            }
        }

    } else if (fault_type == FAULT_TYPE_OV) {

        if (condition_active) {
            if (cell->ov_start_time == 0) {
                cell->ov_start_time = now;  // start timing
            }

            if ((now - cell->ov_start_time) >= grace_ms) {
                if (!cell->ov_active) {
                    BMS_SetCellFault(ic, cell_num, FAULT_TYPE_OV, severity);
                }
            }
        } else {
            cell->ov_start_time = 0; // reset timing
            if (cell->ov_active) {
                BMS_ClearCellFault(ic, cell_num, FAULT_TYPE_OV);
            }
        }

    } else {
        return; // only UV/OV supported here
    }
}

int user_adBms6830_cellVoltageFaults(uint8_t tIC, cell_asic *IC) {
	int total_faults = 0;

//	adBmsWakeupIc(tIC);
//	adBmsReadData(tIC, &IC[0], RDSTATD, Status, D);

	lowest_cell = 100.0;
	int lowest_cell_ind = 0;

	highest_cell = 0.0;
	int highest_cell_ind = 0;

	float sum = 0.0;
	int cell_total = 0;
	for (uint8_t ic = 0; ic < tIC; ic++) {
		uint16_t ov_flags = 0;
		uint16_t uv_flags = 0;
		float voltage;
		for (uint8_t cell_num = 0; cell_num < CELLS_PER_IC; cell_num++) {
			voltage = getVoltage(IC[ic].cell.c_codes[cell_num]);

			if (current_sensor_fault == 0) {
				if (accy_status == READY_POWER) {
					voltage += (current * cell_resistance);
				} else if (accy_status == CHARGE_POWER) {
					voltage -= (current * cell_resistance);
				}
			}

			if (voltage < lowest_cell) {
				lowest_cell = voltage;
				lowest_cell_ind = tIC*ic + cell_num*CELLS_PER_IC;
			}
			if (voltage > highest_cell) {
				highest_cell = voltage;
				highest_cell_ind = tIC*ic + cell_num*CELLS_PER_IC;
			}
			sum += voltage;
			cell_total++;

			// added Debounce Methods
			// --- UV debounce ---
			bool uv_condition = (IC[ic].statd.c_uv[cell_num] || voltage < uv_THRESHOLD);
			if (uv_condition) {
				uv_flags |= (1 << cell_num);
			}

			BMS_DebounceUVOV(ic, cell_num,
							FAULT_TYPE_UV,
							FAULT_SEVERITY_CRITICAL,
							uv_condition,
							CELL_VOLT_GRACE_MS);

			#if PRINT_ON
			if (uv_condition) {
				printf("HW UV (pending/debounced): IC%d C%d = %.3fV\n", ic, cell_num, voltage);
			}
			#endif


			// --- OV debounce ---
			bool ov_condition = (IC[ic].statd.c_ov[cell_num] || voltage > ov_THRESHOLD);
			if (ov_condition) {
				ov_flags |= (1 << cell_num);
			}

			BMS_DebounceUVOV(ic, cell_num,
							FAULT_TYPE_OV,
							FAULT_SEVERITY_CRITICAL,
							ov_condition,
							CELL_VOLT_GRACE_MS);

			#if PRINT_ON
			if (ov_condition) {
				printf("HW OV (pending/debounced): IC%d C%d = %.3fV\n", ic, cell_num, voltage);
			}
			#endif

		}


		// Store hardware flags for reference
		BMS_Faults.hw_ov_flags[ic] = ov_flags;
		BMS_Faults.hw_uv_flags[ic] = uv_flags;
	}

	avg_cell = (cell_total > 0) ? (sum / cell_total) : 0.0f;
	delta_cell = highest_cell - lowest_cell;
	if (abs(((sum - highest_cell)/cell_total) - highest_cell) > MAX_VOLTAGE_DELTA) {
		BMS_SetCellFault(highest_cell_ind/10, highest_cell_ind%10, FAULT_TYPE_DELTA, FAULT_SEVERITY_WARNING);
		total_faults++;
	}


	if (abs(((sum - lowest_cell)/cell_total) - lowest_cell) > MAX_VOLTAGE_DELTA) {
		BMS_SetCellFault(lowest_cell_ind/10, lowest_cell_ind%10, FAULT_TYPE_DELTA, FAULT_SEVERITY_WARNING);
		total_faults++;
	}

	total_faults = BMS_GetFaultCount(FAULT_CATEGORY_VOLTAGE);

	if (total_faults == 0) {
		cell_fault = 0;
	}

    return total_faults;
}
