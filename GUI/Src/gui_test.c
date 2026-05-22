/*
 * gui_test.c
 *
 *  Created on: Apr 4, 2026
 *      Author: ishanchitale
 */

#include "gui_test.h"
#include "voltage_calculations.h"
#include "adBms_Application.h"
#include "balancing.h"
#include "j_plug.h"
#include "charging.h"
#include "elcon_charger.h"

char json_buf[4096];

/**
 * @brief  Send balancing status telemetry over UART to the GUI.
 *
 * Format:  BAL:STATE=<state>,PCT=<percent>,IC<n>_DCC=0x<hex>,...\n
 *
 * Example: BAL:STATE=DISCHARGE,PCT=50,IC0_DCC=0x03FF,IC1_DCC=0x0000\n
 *
 * The GUI Python backend parses this line to display real-time balancing
 * state and per-IC discharge control (DCC) bitmasks in the frontend.
 */
void send_bal_status(uint8_t tIC, cell_asic *ic) {
    const char *state_str;
    switch (balance_state) {
        case BALANCE_IDLE:              state_str = "IDLE"; break;
        case BALANCE_COMPUTE_DISCHARGE: state_str = "COMPUTE"; break;
        case BALANCE_DISCHARGE:         state_str = "DISCHARGE"; break;
        case BALANCE_WAIT:              state_str = "WAIT"; break;
        case BALANCE_COMPLETE:          state_str = "COMPLETE"; break;
        default:                        state_str = "UNKNOWN"; break;
    }

    int offset = 0;
    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
        "BAL:STATE=%s,PCT=%u", state_str, getBalancePercent());

    for (uint8_t i = 0; i < tIC; ++i) {
        offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
            ",IC%u_DCC=0x%04X", i, ic[i].tx_cfgb.dcc);
    }

    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, "\n");

    HAL_UART_Transmit(&hlpuart1, (uint8_t*)json_buf, offset, HAL_MAX_DELAY);
}

void send_temp_status(void) {
    int offset = 0;
    osMutexWait(TEMP_MUTEXHandle, osWaitForever);
    for (uint8_t i = 0; i < TOTAL_IC; ++i) {
        offset = snprintf(json_buf, sizeof(json_buf), "IC%u_TMP:", i + 1);
        for (uint8_t j = 0; j < CELLS_PER_IC; ++j) {
            float t = temp_context.temp_conversions[i][j];
            if (j == 0) {
                offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, "T%u=%.1f", j + 1, t);
            } else {
                offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, ",T%u=%.1f", j + 1, t);
            }
        }
        offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, "\n");
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)json_buf, offset, HAL_MAX_DELAY);
    }
    osMutexRelease(TEMP_MUTEXHandle);
}

void send_charger_status(void) {
    char chg_buf[256];
    const char* pp_str;
    switch(proximity_pilot_state) {
        case STATE_PP_CONNECTED: pp_str = "CONNECTED"; break;
        case STATE_PP_BUTTON_PRESSED: pp_str = "BTN_PRESSED"; break;
        case STATE_PP_NOT_CONNECTED: pp_str = "DISCONNECTED"; break;
        default: pp_str = "UNKNOWN"; break;
    }
    
    uint32_t duty = 0;
    if (j1772_context.control_pilot_period > 0) {
        duty = (j1772_context.control_pilot_pulse * 100) / j1772_context.control_pilot_period;
    }
    if (control_pilot_state != STATE_CP_PWM_PRESENT) {
        duty = 0;
    }
    
    const char* chg_state_str;
    switch(charging_state) {
        case CHG_IDLE: chg_state_str = "IDLE"; break;
        case CHG_WAITING: chg_state_str = "WAITING"; break;
        case CHG_ACTIVE: chg_state_str = "CHARGING_ACTIVE"; break;
        case CHG_ELCON_FAULT: chg_state_str = "CHARGER_FAULT"; break;
        case CHG_COMPLETE: chg_state_str = "CHARGER_COMPLETE"; break;
        default: chg_state_str = "UNKNOWN"; break;
    }

	snprintf(chg_buf, sizeof(chg_buf), "CHG:PP=%s,CP=%lu,AD=%.1f,V=%u,I=%u|%s\n",
        pp_str, duty, advertised_amps_dc, 
        elcon_charger_context.chgmsg_18FF50E5_DF.data.charger_output_voltage,
        elcon_charger_context.chgmsg_18FF50E5_DF.data.charger_output_current,
        chg_state_str);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)chg_buf, strlen(chg_buf), HAL_MAX_DELAY);
}
