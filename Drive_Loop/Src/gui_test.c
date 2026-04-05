/*
 * gui_test.c
 *
 *  Created on: Apr 4, 2026
 *      Author: ishanchitale
 */

#include "gui_test.h"
#include <stdio.h>
#include <string.h>

char json_buf[JSON_BUF_SIZE];

/* Map BMS_STATE enum to a string for the GUI */
static const char* bms_state_str(BMS_STATE st)
{
	switch(st)
	{
		case BMS_IDLE:        return "IDLE";
		case BMS_FAULT:       return "FAULT";
		case BMS_CHARGING:    return "CHARGING";
		case BMS_BALANCING:   return "BALANCING";
		case BMS_PRECHARGING: return "PRECHARGING";
		case BMS_DRIVE:       return "DRIVE";
		default:              return "UNKNOWN";
	}
}

int build_bms_json(void)
{
	int offset = 0;
	int remaining = JSON_BUF_SIZE;

	/* ── Pack-level stats ── */
	offset += snprintf(json_buf + offset, remaining - offset,
		"{\"pack_voltage\":%.1f,"
		"\"current\":%.1f,"
		"\"soc\":%.0f,"
		"\"max_temp\":%.1f,"
		"\"charging_state\":\"%s\","
		"\"balancing_enabled\":%s,"
		"\"ccl\":%.1f,"
		"\"dcl\":%.1f,",
		bms_pack_voltage,
		current_sensor_val,
		soc,
		highest_cell_temp,
		bms_state_str(bms_state),
		(bms_state == BMS_BALANCING) ? "true" : "false",
		ccl,
		dcl
	);

	/* ── Faults array ── */
	offset += snprintf(json_buf + offset, remaining - offset, "\"faults\":[");
	{
		int first_fault = 1;
		for(int i = 0; i < TOTAL_IC; i++)
		{
			for(int j = 0; j < CELLS_PER_IC; j++)
			{
				int cell_num = i * CELLS_PER_IC + j + 1;
				float v = voltage_conversions[i][j];

				if(v >= OV_THRESHOLD)
				{
					if(!first_fault) offset += snprintf(json_buf + offset, remaining - offset, ",");
					offset += snprintf(json_buf + offset, remaining - offset,
						"{\"cell\":%d,\"type\":\"OV\",\"voltage\":%.3f}", cell_num, v);
					first_fault = 0;
				}
				else if(v <= UV_THRESHOLD && v > 0.1f)
				{
					if(!first_fault) offset += snprintf(json_buf + offset, remaining - offset, ",");
					offset += snprintf(json_buf + offset, remaining - offset,
						"{\"cell\":%d,\"type\":\"UV\",\"voltage\":%.3f}", cell_num, v);
					first_fault = 0;
				}
			}
		}
	}
	offset += snprintf(json_buf + offset, remaining - offset, "],");

	/* ── Segments array ── */
	offset += snprintf(json_buf + offset, remaining - offset, "\"segments\":[");

	for(int seg = 0; seg < NUM_SEGMENTS; seg++)
	{
		if(seg > 0) offset += snprintf(json_buf + offset, remaining - offset, ",");

		offset += snprintf(json_buf + offset, remaining - offset,
			"{\"id\":%d,\"ics\":[", seg + 1);

		for(int ic_local = 0; ic_local < ICS_PER_SEGMENT; ic_local++)
		{
			int ic_global = seg * ICS_PER_SEGMENT + ic_local;
			if(ic_global >= TOTAL_IC) break; /* Don't exceed actual IC count */
			if(ic_local > 0) offset += snprintf(json_buf + offset, remaining - offset, ",");

			/* IC object */
			offset += snprintf(json_buf + offset, remaining - offset,
				"{\"id\":%d,\"cells\":[", ic_local);

			/* Cell voltages */
			for(int c = 0; c < CELLS_PER_IC; c++)
			{
				if(c > 0) offset += snprintf(json_buf + offset, remaining - offset, ",");
				offset += snprintf(json_buf + offset, remaining - offset,
					"%.3f", voltage_conversions[ic_global][c]);
			}

			offset += snprintf(json_buf + offset, remaining - offset, "],\"temps\":[");

			/* Cell temps */
			for(int c = 0; c < CELLS_PER_IC; c++)
			{
				if(c > 0) offset += snprintf(json_buf + offset, remaining - offset, ",");
				offset += snprintf(json_buf + offset, remaining - offset,
					"%.1f", temp_conversions[ic_global][c]);
			}

			/* DCC bitmask + OV/UV flags */
			uint16_t dcc_mask = IC[ic_global].tx_cfgb.dcc;
			uint16_t ov_flags = 0;
			uint16_t uv_flags = 0;

			for(int c = 0; c < CELLS_PER_IC; c++)
			{
				float v = voltage_conversions[ic_global][c];
				if(v >= OV_THRESHOLD) ov_flags |= (1 << c);
				if(v <= UV_THRESHOLD && v > 0.1f) uv_flags |= (1 << c);
			}

			offset += snprintf(json_buf + offset, remaining - offset,
				"],\"dcc\":%u,\"ov_flags\":%u,\"uv_flags\":%u}",
				dcc_mask, ov_flags, uv_flags);
		}

		offset += snprintf(json_buf + offset, remaining - offset, "]}");
	}

	offset += snprintf(json_buf + offset, remaining - offset, "]}\n");

	return offset;
}
