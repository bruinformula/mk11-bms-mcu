#include "adbms_can_helper.h"
#include "custom_functions.h"
#include "serialPrintResult.h"

void populate_CAN1(CAN1_DATAFRAME *frame, cell_asic *ICs, int totalIC) {
    // --- PACK CURRENT ---
    frame->data.pack_current = (int16_t)(current * 10.0f); // current is extern, *0.1 A for CAN

    // --- PACK VOLTAGE ---
    float pack_voltage_sum = 0.0f;
    // for (int i = 0; i < totalIC; i++) {
    //     for (int j = 0; j < NUM_CELLS_PER_IC; j++) {
    //         float cell_voltage = getVoltage(ICs[i].cell.c_codes[j]);
    //         pack_voltage_sum += cell_voltage;
    //     }
    // }
    pack_voltage_sum = getPackVoltage(totalIC, ICs);
    frame->data.pack_voltage = (uint16_t)(pack_voltage_sum * 10.0f); // *0.1 V for CAN

    // --- STATE OF CHARGE (SOC) ---
    soc = updateSOC(); // (pack_voltage_sum - 280.0f) / (420.0f - 280.0f); // scale 280V–420V

    frame->data.pack_soc = (uint8_t)(soc); // *0.5% for 0–100% (0–200 steps)

    // --- RELAY + SYSTEM STATUS FLAGS ---
    frame->data.discharge_relay = 1; // TODO: replace with actual GPIO read if needed
    frame->data.charge_relay    = 1;
    frame->data.charger_safety  = 0; // Optional safety flag

    // --- Fault Indicator (MIL light) ---
    frame->data.mil_state = (cell_fault || temp_fault) ? 1 : 0;

    // --- Charging Status ---
    frame->data.charging_on = (accy_status == CHARGE_POWER) ? 1 : 0;
    frame->data.is_ready    = (accy_status == READY_POWER) ? 1 : 0;
    frame->data.always_on   = 1;

    // --- MPx/MPEnable Flags (dummy/stub logic) ---
    frame->data.mp_enable   = 1;
    frame->data.mpo1_state  = 0;
    frame->data.mpo2_state  = 1;
    frame->data.mpo3_state  = 0;
    frame->data.mpo4_state  = 1;
    frame->data.mpi1_state  = 0;
    frame->data.mpi2_state  = 1;
    frame->data.mpi3_state  = 0;

    // --- Checksum (optional, unused for now) ---
    frame->data.checksum = 0;
}


void populate_CAN2(CAN2_DATAFRAME *frame, cell_asic *ICs, int totalIC) {
    // --- Current Limits ---
    frame->data.pack_dcl = dcl;
    frame->data.pack_ccl = ccl;

    // --- Temp Aggregation ---
    // float max_temp = -999.0f;
    // float min_temp = 999.0f;

    // for (int i = 0; i < totalIC; i++) {
    // 	for (int j = 0; j < NUM_TEMPS_PER_IC; j++) {
    //         float voltage = getVoltage(ICs[i].aux.a_codes[j]);

    //         // Skip invalid readings (optional)
    //         if (voltage < 0.1f || voltage > 4.0f) continue;

    //         float temp = -225.6985f * voltage * voltage * voltage
    //                      + 1310.5937f * voltage * voltage
    //                      - 2594.7697f * voltage
    //                      + 1767.8260f;

    //         if (temp > max_temp) max_temp = temp;
    //         if (temp < min_temp) min_temp = temp;
    //     }
    // }

    // Clamp to int8 range
    // if (max_temp > 127) max_temp = 127;
    // if (min_temp < -128) min_temp = -128;
    if (highest_temp > 127) lowest_temp = 127;
    if (highest_temp < -128) highest_temp = -128;
    if (lowest_temp > 127) lowest_temp = 127;
    if (lowest_temp < -128) lowest_temp = -128;

    frame->data.pack_high_temp = (int8_t)highest_temp;
    frame->data.pack_low_temp = (int8_t)lowest_temp;

    // --- Padding / Checksum ---
    frame->data.reserved0 = 0;
    frame->data.reserved1 = 0;
    frame->data.checksum = 0; // optional
}

void populate_CAN3(CAN3_DATAFRAME *frame, cell_asic *ICs, int totalIC) {
    float min_voltage = 1e6f;  // large initial value for comparison
    float max_voltage = -1e6f; // small initial value for comparison
    int cell_count = 0;
    // for (int i = 0; i < totalIC; i++) {
    //     for (int j = 0; j < NUM_CELLS_PER_IC; j++) {
    //         float voltage = getVoltage(ICs[i].cell.c_codes[j]);

    //         // Count valid cells processed
    //         cell_count++;

    //         if (voltage < min_voltage) min_voltage = voltage;
    //         if (voltage > max_voltage) max_voltage = voltage;
    //     }
    // }

    min_voltage = lowest_cell;
    max_voltage = highest_cell;

    // Fallback if no valid cells were processed
    if (cell_count == 0) {
        min_voltage = 0.0f;
        max_voltage = 0.0f;
    }

    // Convert to 0.0001 V units for CAN message
    uint16_t min_mv = (uint16_t)(min_voltage * 10000.0f);
    uint16_t max_mv = (uint16_t)(max_voltage * 10000.0f);

    frame->data.low_cell_voltage  = min_mv;
    frame->data.high_cell_voltage = max_mv;

    frame->data.reserved0 = 0;
    frame->data.reserved1 = 0;
    frame->data.reserved2 = 0;
    frame->data.checksum  = 0;
}

void populate_charge_CAN(FDCAN_CHARGER_CONTEXT *CHARGER_CONTEXT, cell_asic *ICs, int totalIC) {
	//set pack current data
	CHARGER_CONTEXT->chgmsg_1806e7f4.data.pack_voltage = (int16_t)(getPackVoltage(totalIC, ICs) * 10.0f); // current is extern, *0.1 A for CAN
	CHARGER_CONTEXT->chgmsg_1806e7f4.data.pack_ccl = (int16_t)(calcCCL() * 10.0f); // current is extern, *0.1 A for CAN
	//todo: dont forget charge_enable
	CHARGER_CONTEXT->chgmsg_1806e7f4.data.charge_enable = !is_charging;
//	CHARGER_CONTEXT->chgmsg_1806e7f4.data.charge_enable = !HAL_GPIO_ReadPin(SDC_IN_GPIO_Port,SDC_IN_Pin);
	CHARGER_CONTEXT->chgmsg_1806e7f4.data.reserved0 = 0;
	CHARGER_CONTEXT->chgmsg_1806e7f4.data.reserved1= 0;
	CHARGER_CONTEXT->chgmsg_1806e7f4.data.reserved2 = 0;


	CHARGER_CONTEXT->chgmsg_1806e5f4.data.high_cell_voltage = (int16_t)(highest_cell * 10.0f); // current is extern, *0.1 A for CAN
	CHARGER_CONTEXT->chgmsg_1806e5f4.data.pack_ccl = (int16_t)(calcCCL() * 10.0f); // current is extern, *0.1 A for CAN
	CHARGER_CONTEXT->chgmsg_1806e5f4.data.reserved0 = 0;
	CHARGER_CONTEXT->chgmsg_1806e5f4.data.reserved1 = 0;
	CHARGER_CONTEXT->chgmsg_1806e5f4.data.reserved2 = 0;
	CHARGER_CONTEXT->chgmsg_1806e5f4.data.reserved3 = 0;


	CHARGER_CONTEXT->chgmsg_1806e9f4.data.high_cell_voltage = (int16_t)(highest_cell * 10.0f); // current is extern, *0.1 A for CAN
	CHARGER_CONTEXT->chgmsg_1806e9f4.data.pack_ccl = (int16_t)(calcCCL() * 10.0f); // current is extern, *0.1 A for CAN
	CHARGER_CONTEXT->chgmsg_1806e9f4.data.reserved0 = 0;
	CHARGER_CONTEXT->chgmsg_1806e9f4.data.reserved1 = 0;
	CHARGER_CONTEXT->chgmsg_1806e9f4.data.reserved2 = 0;
	CHARGER_CONTEXT->chgmsg_1806e9f4.data.reserved3 = 0;



	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved0 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved1 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved2 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved3 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved4 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved5 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved6 = 0;
	CHARGER_CONTEXT->chgmsg_18ff50e5.data.reserved7 = 0;

}

