/*
 * gui_test.c
 *
 *  Created on: Apr 4, 2026
 *      Author: ishanchitale
 */

#include "gui_test.h"

char json_buf[4096];

int build_bms_json() {
    int offset = 0;

    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
        "{\"voltages\":[");

    // Voltages
    for (int i = 0; i < CELLS_PER_IC; i++) {
        offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
            "%.3f", voltage_conversions[0][i]);

        if (i < CELLS_PER_IC  - 1)
            offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, ",");
    }

    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
        "],\"temperatures\":[");

    // Temperatures
    for (int i = 0; i < CELLS_PER_IC; i++) {
        offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
            "%.2f", temp_conversions[0][i]);

        if (i < CELLS_PER_IC - 1)
            offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, ",");
    }

    // Stats + limits
    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
        "],\"ccl\":%.2f,\"dcl\":%.2f,"
        "\"voltage_stats\":{\"min\":%.3f,\"max\":%.3f,\"avg\":%.3f},"
        "\"temperature_stats\":{\"min\":%.2f,\"max\":%.2f,\"avg\":%.2f}"
        "}\n",
        ccl, dcl,
        lowest_cell_voltage, highest_cell_voltage, avg_cell_voltage,
        lowest_cell_temp, highest_cell_temp, avg_cell_temp
    );

    return offset;  // number of bytes to send
}

void send_bms_json() {
	int len = build_bms_json();
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)json_buf, len, HAL_MAX_DELAY);
}
