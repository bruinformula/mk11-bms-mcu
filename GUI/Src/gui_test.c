/*
 * gui_test.c
 *
 *  Created on: Apr 4, 2026
 *      Author: ishanchitale
 */

#include "gui_test.h"

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
