#ifndef INC_ADBMS_CAN_HELPER_H_
#define INC_ADBMS_CAN_HELPER_H_

#include "dataframes.h"
#include "stm32g4xx_hal.h"
#include "adBms6830Data.h"  // Ensure this defines `cell_asic`
#include "can.h"

// Updated to match .c definitions: totalIC is int, ICs is cell_asic *
void populate_CAN1(CAN1_DATAFRAME *frame, cell_asic *ICs, int totalIC);
void populate_CAN2(CAN2_DATAFRAME *frame, cell_asic *ICs, int totalIC);
void populate_CAN3(CAN3_DATAFRAME *frame, cell_asic *ICs, int totalIC);
void populate_CAN4(CAN4_DATAFRAME *frame, cell_asic *ICs, int totalIC, bool precharge);
void populate_charge_CAN(FDCAN_CHARGER_CONTEXT *CHARGER_CONTEXT, cell_asic *ICs, int totalIC);

#endif /* INC_ADBMS_CAN_HELPER_H_ */
