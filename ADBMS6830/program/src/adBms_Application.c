/*******************************************************************************
 Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 This software is proprietary & confidential to Analog Devices, Inc.
 and its licensor.
 ******************************************************************************
 * @file:    adbms_Application.c
 * @brief:   adbms application test cases
 * @version: $Revision$
 * @date:    $Date$
 * Developed by: ADIBMS Software team, Bangalore, India
 *****************************************************************************/
/*! \addtogroup APPLICATION
 *  @{
 */

/*! @addtogroup Application
 *  @{
 */
#include "common.h"
#include "adbms_main.h"
#include "adBms_Application.h"
#include "adBms6830CmdList.h"
#include "adBms6830GenericType.h"
#include "serialPrintResult.h"
#include "mcuWrapper.h"
#include <math.h>
#include "adbms_can_helper.h"
#include "custom_functions.h"
#ifdef MBED
extern Serial pc;
#endif
/**
 *******************************************************************************
 * @brief Setup Variables
 * The following variables can be modified to configure the software.
 *******************************************************************************
 */

void fakeChargeParams(void);
void fakeDriveParams(void);

#define TOTAL_IC 1
#define IC_CHUNK 1
cell_asic IC[TOTAL_IC];
cell_asic TEMP_IC[IC_CHUNK];

/* ADC Command Configurations */
RD REDUNDANT_MEASUREMENT = RD_OFF;
CH AUX_CH_TO_CONVERT = AUX_ALL;
CONT CONTINUOUS_MEASUREMENT = SINGLE;
OW_C_S CELL_OPEN_WIRE_DETECTION = OW_OFF_ALL_CH;
OW_AUX AUX_OPEN_WIRE_DETECTION = AUX_OW_OFF;
PUP OPEN_WIRE_CURRENT_SOURCE = PUP_DOWN;
DCP DISCHARGE_PERMITTED = DCP_OFF;
RSTF RESET_FILTER = RSTF_OFF;
ERR INJECT_ERR_SPI_READ = WITHOUT_ERR;

/*Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS*/
LOOP_MEASURMENT MEASURE_CELL = ENABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_AVG_CELL = DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_F_CELL = DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_S_VOLTAGE = DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_AUX = DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_RAUX = DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_STAT = DISABLED; /*   This is ENABLED or DISABLED       */

/* ==== Balancing ==== */
int8_t balancing = 0; // 0: No balancing, 1: Balancing
float target_lowest_cell = -1;
float internal_resistance = 0.42;
uint8_t readSegment = 0;

uint16_t multiMask = 0;
uint16_t tick = 0;


void adbms_main(int command) {
	// printMenu();
	//   adBms6830_init_config(TOTAL_IC, &IC[0]);
	// #ifdef MBED
	//   // pc.printf("Waiting for input... \n");
	// #else
	//   // printf("Waiting for input... \n");
	// #endif
	//   while(1)
	//   {
	//     uint8_t user_command;
	// // #ifdef MBED
	// //   pc.printf("Inside the while loop! \n");
	// // #else
	// //   printf("Hello... \n");
	// // #endif
	// #ifdef MBED
	//     // pc.scanf("%d", &user_command);
	//     // pc.printf("Enter cmd:%d\n", user_command);
	// #else
	//     // printf("Scanning... \n");
	//     fflush(stdout);
	//     user_command = command;
	//     // printf("Received: %c\n", user_command);
	//     // printf("Enter cmd:%d\n", user_command);
	// #endif
	// run_command(user_command);
	if (accy_status == -1 || accy_status == 0) {
		// If the accessory is not in charge or ready state, set the accumulator status
		user_adBms6830_getAccyStatus(); // Determine whether the accy is in charge or ready state
	}

	readSegment += IC_CHUNK;
	if (readSegment >= TOTAL_IC) {
		readSegment = 0;
	}
	//	TEMP_IC[0] = IC[readSegment*2];
	//	TEMP_IC[1] = IC[(readSegment*2)+1];

//	    populateIC(TEMP_IC, 2);
//	populateIC(&IC[0], TOTAL_IC);

	populateIC(&IC[readSegment], IC_CHUNK);

	if (accy_status == READY_POWER && !cell_fault && !temp_fault) {
		user_adBms6830_setFaults(); // Check for faults and set GPIO pins accordingly
		getPackVoltage(TOTAL_IC, &IC[0]); // Get the pack voltage
		// getCurrentSensorData();               // Get the current sensor data  ==> moved into populateIC
		updateSOC();  // Get the state of charge (SOC) based on the pack voltage
		ccl = calcCCL();                      // get charge current limit
		dcl = calcDCL();                      // get discharge current limit



		uint32_t now = HAL_GetTick();

	} else if (accy_status == CHARGE_POWER && !cell_fault && !temp_fault) {
		uint32_t timingshits = HAL_GetTick();
		if (PRINT_ON) printf("time to read once: %.2f", HAL_GetTick() - timingshits);

		user_adBms6830_setFaults(); // Check for faults and set GPIO pins accordingly
		getPackVoltage(TOTAL_IC, &IC[0]); // Get the pack voltage
		// getCurrentSensorData();               // Get the current sensor data   ==> moved into populateIC
		updateSOC();  // Get the state of charge (SOC) based on the pack voltage
		ccl = calcCCL();                      // get charge current limit
		dcl = calcDCL();                      // get discharge current limit
		//    fanPWMControl(highest_temp, htimPWM); // Control the fan based on the highest temperature


		uint32_t now = HAL_GetTick();
	} else {
		if (PRINT_ON) printf("Fan Status: %.2f\n", fan_status);
		// fanPWMControl(42.0f, htimPWM);
		ccl = calcCCL();                      // get charge current limit
		dcl = calcDCL();
		user_adBms6830_setFaults(); // Check for faults and set GPIO pins accordingly
		getPackVoltage(TOTAL_IC, &IC[0]); // Get the pack voltage
		// getCurrentSensorData();           // Get the current sensor data   ==> moved into populateIC
		updateSOC(); // Get the state of charge (SOC) based on the pack voltage

		if (!cell_fault && !temp_fault && balancing == 1) {
			// If there are no faults, balance the cells
			balanceCells(TOTAL_IC, IC, PWM_100_0_PCT); // TODO: Set the duty cycle for balancing
		} else if (balancing == 1) {
			balancing = 0;               // Stop balancing if there are faults
			stopBalancing(TOTAL_IC, IC); // Stop balancing if there are faults
		}

		if (PRINT_ON) printReadPwmDutyCycle(TOTAL_IC, IC, PWMA, ALL_GRP);
		if (PRINT_ON) printWritePwmDutyCycle(TOTAL_IC, IC, PWMA, ALL_GRP); // Print the PWM duty cycle for debugging

		// printPwmRegisters(TOTAL_IC, IC); // Print the PWM registers for debugging

		tick++;


		uint32_t now = HAL_GetTick();

	}

	if (PRINT_ON) {
		printVoltages(TOTAL_IC, &IC[0], Cell);

		printf("accy_status: %d, balancing: %d\n", accy_status, balancing);
		printf("Pack Voltage: %.2fV, SOC: %.2f\n",
				getPackVoltage(TOTAL_IC, &IC[0]), soc);
		printf("Lowest V: %.2fV, ", lowest_cell);
		printf("Highest V: %.2fV, ", highest_cell);
		printf("Average V: %.2fV, ", avg_cell);
		printf("Target lowest V: %.2fV\n", target_lowest_cell);
		printf("Lowest temp: %.2fC, ID: %d", lowest_temp, lowest_temp_ID);
		printf("Highest temp: %.2fC, ID: %d", highest_temp, highest_temp_ID);
		printf("Average temp: %.2fC\n\n", avg_temp);
		print_fault_summary(); // Print the fault summary
	}
	// else {
	//   populateIC(IC, TOTAL_IC);
	//   user_adBms6830_setFaults(); // Check for faults and set GPIO pins accordingly
	//   getPackVoltage(TOTAL_IC, &IC[0]); // Get the pack voltage
	//   getCurrentSensorData(); // Get the current sensor data
	//   updateSOC(); // Get the state of charge (SOC) based on the pack voltage
	//   fanPWMControl(highest_temp, htimPWM); // Control the fan based on the highest temperature
	//   populate_CAN1(&ctx->msg_6b0, &IC[0], TOTAL_IC); // Populate CAN frames with data (TODO: check with justin)
	//   populate_CAN2(&ctx->msg_6b1, &IC[0], TOTAL_IC);
	//   populate_CAN3(&ctx->msg_6b2, &IC[0], TOTAL_IC);
	//   uint32_t now = HAL_GetTick();
	//   FDCAN_BMS_Mailman(hfdcan, ctx, now);
	//   Delay_ms(2000);
	// }

	//    Delay_ms(3000);
	//
	//
	//    // Populate data into CAN frames
	//
	//    populate_CAN1(&ctx->msg_6b0, &IC[0], TOTAL_IC);
	//    populate_CAN2(&ctx->msg_6b1, &IC[0], TOTAL_IC);
	//    populate_CAN3(&ctx->msg_6b2, &IC[0], TOTAL_IC);
	//
	//
	//    // Transmit via mailman
	//    uint32_t now = HAL_GetTick();
	//    FDCAN_BMS_Mailman(hfdcan, ctx, now);

	//    getCurrentSensorData();
}

void run_command(int cmd) {
	adBms6830_start_adc_cell_voltage_measurment(TOTAL_IC);
	Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	adBms6830_read_cell_voltages(TOTAL_IC, &IC[0]);
	int c_fault = user_adBms6830_cellFault(TOTAL_IC, &IC[0]);
	if (c_fault != 0) {
		cell_fault = c_fault;
	}
	Delay_ms(8);
	adBms6830_start_aux_voltage_measurment(TOTAL_IC, &IC[0]);
	Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	adBms6830_read_aux_voltages(TOTAL_IC, &IC[0]);
	int t_fault = user_adBms6830_tempFault(TOTAL_IC, &IC[0]);
	if (t_fault == 1) {
		temp_fault = CELL_TEMP_FAULT;
	}
	// Delay_ms(8);

	// adBms6830_start_avgcell_voltage_measurment(TOTAL_IC);
	// Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	// adBms6830_read_avgcell_voltages(TOTAL_IC, &IC[0]);
	// Delay_ms(8);

	getCurrentSensorData();

	switch(cmd)
	   {

	   case 1:
	     adBms6830_write_read_config(TOTAL_IC, &IC[0]);
	     break;

	   case 2:
	     adBms6830_read_config(TOTAL_IC, &IC[0]);
	     break;

	   case 3:
	     adBms6830_start_adc_cell_voltage_measurment(TOTAL_IC);
	     break;

	   case 4:
	     adBms6830_read_cell_voltages(TOTAL_IC, &IC[0]);
	     break;

	   case 5:
	     adBms6830_start_adc_s_voltage_measurment(TOTAL_IC);
	     break;

	   case 6:
	     adBms6830_read_s_voltages(TOTAL_IC, &IC[0]);
	     break;

	   case 7:
	     adBms6830_start_avgcell_voltage_measurment(TOTAL_IC);
	     break;

	   case 8:
	     adBms6830_read_avgcell_voltages(TOTAL_IC, &IC[0]);
	     break;

	   case 9:
	     adBms6830_start_fcell_voltage_measurment(TOTAL_IC);
	     break;

	   case 10:
	     adBms6830_read_fcell_voltages(TOTAL_IC, &IC[0]);
	     break;

	   case 11:
	     adBms6830_start_aux_voltage_measurment(TOTAL_IC, &IC[0]);
	     break;

	   case 12:
	     adBms6830_read_aux_voltages(TOTAL_IC, &IC[0]);
	     break;

	   case 13:
	     adBms6830_start_raux_voltage_measurment(TOTAL_IC, &IC[0]);
	     break;

	   case 14:
	     adBms6830_read_raux_voltages(TOTAL_IC, &IC[0]);
	     break;

	   case 15:
	     adBms6830_read_status_registers(TOTAL_IC, &IC[0]);
	     break;

	   case 16:
	     loop_count = 0;
	     adBmsWakeupIc(TOTAL_IC);
	     adBmsWriteData(TOTAL_IC, &IC[0], WRCFGA, Config, A);
	     adBmsWriteData(TOTAL_IC, &IC[0], WRCFGB, Config, B);
	     adBmsWriteData(TOTAL_IC, &IC[0], WRCFGB, Config, C);
	     adBmsWakeupIc(TOTAL_IC);
	     adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	     Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	     while(loop_count < LOOP_MEASUREMENT_COUNT)
	     {
	       measurement_loop();
	       Delay_ms(MEASUREMENT_LOOP_TIME);

	       loop_count = loop_count + 1;
	     }
	     //printMenu();
	     Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	     break;

	   case 17:
	     adBms6830_clear_cell_measurement(TOTAL_IC);
	     break;

	   case 18:
	     adBms6830_clear_aux_measurement(TOTAL_IC);
	     break;

	   case 19:
	     adBms6830_clear_spin_measurement(TOTAL_IC);
	     break;

	   case 20:
	     adBms6830_clear_fcell_measurement(TOTAL_IC);
	     break;

	   case 21:
	     adBms6830_write_config(TOTAL_IC, &IC[0]);
	     break;

	   case 0:
	     printMenu();

	     break;

	   default:
	 #ifdef MBED
	     pc.printf("Incorrect Option\n\n");
	 #else
	     printf("Incorrect Option\n\n");
	 #endif
	     break;
	   }
}

/**
 *******************************************************************************
 * @brief Set configuration register A. Refer to the data sheet
 *        Set configuration register B. Refer to the data sheet
 *******************************************************************************
 */
void adBms6830_init_config(uint8_t tIC, cell_asic *ic) {
	for (uint8_t cic = 0; cic < tIC; cic++) {
		/* Init config A */
		ic[cic].tx_cfga.refon = PWR_UP;
		//    ic[cic].cfga.cth = CVT_8_1mV;
		//    ic[cic].cfga.flag_d = ConfigA_Flag(FLAG_D0, FLAG_SET) | ConfigA_Flag(FLAG_D1, FLAG_SET);
		//    ic[cic].cfga.gpo = ConfigA_Gpo(GPO2, GPO_SET) | ConfigA_Gpo(GPO10, GPO_SET);
		ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
		//    ic[cic].cfga.soakon = SOAKON_CLR;
		//    ic[cic].cfga.fc = IIR_FPA256;

		/* Init config B */
		//    ic[cic].cfgb.dtmen = DTMEN_ON;
		ic[cic].tx_cfgb.vov = SetOverVoltageThreshold(OV_THRESHOLD);
		ic[cic].tx_cfgb.vuv = SetUnderVoltageThreshold(UV_THRESHOLD);

		ic[cic].tx_cfgc.cell_en = 0xFFFF;
		//    ic[cic].cfgb.dcc = ConfigB_DccBit(DCC16, DCC_BIT_SET);
		//    SetConfigB_DischargeTimeOutValue(tIC, &ic[cic], RANG_0_TO_63_MIN, TIME_1MIN_OR_0_26HR);
	}
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
	adBmsWriteData(tIC, &ic[0], WRCFGC, Config, C);
}

/**
 *******************************************************************************
 * @brief Write and Read Configuration Register A/B
 *******************************************************************************
 */
void adBms6830_write_read_config(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
	adBmsWriteData(tIC, &ic[0], WRCFGC, Config, C);

	adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
	adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
	adBmsReadData(tIC, &ic[0], RDCFGC, Config, C);
	// printWriteConfig(tIC, &ic[0], Config, ALL_GRP);
	// printReadConfig(tIC, &ic[0], Config, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Write Configuration Register A/B
 *******************************************************************************
 */
void adBms6830_write_config(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
	adBmsWriteData(tIC, &ic[0], WRCFGC, Config, C);
	// printWriteConfig(tIC, &ic[0], Config, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Read Configuration Register A/B
 *******************************************************************************
 */
void adBms6830_read_config(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
	adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
	adBmsReadData(tIC, &ic[0], RDCFGC, Config, C);
	// printReadConfig(tIC, &ic[0], Config, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Start ADC Cell Voltage Measurement
 *******************************************************************************
 */
void adBms6830_start_adc_cell_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
#ifdef MBED
	// pc.printf("Cell conversion completed\n");
#else
	// printf("Cell conversion completed\n");
#endif
}

/**
 *******************************************************************************
 * @brief Read Cell Voltages
 *******************************************************************************
 */
void adBms6830_read_cell_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	HAL_Delay(100);
	adBms6830_Snap();
	adBmsReadData(tIC, &ic[0], RDCVA, Cell, A);
	adBmsReadData(tIC, &ic[0], RDCVB, Cell, B);
	adBmsReadData(tIC, &ic[0], RDCVC, Cell, C);
	adBmsReadData(tIC, &ic[0], RDCVD, Cell, D);
	adBmsReadData(tIC, &ic[0], RDCVE, Cell, E);
	adBmsReadData(tIC, &ic[0], RDCVF, Cell, F);
	adBms6830_Unsnap();

}

/**
 *******************************************************************************
 * @brief Start ADC S-Voltage Measurement
 *******************************************************************************
 */
void adBms6830_start_adc_s_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS, DCP_OFF, OW_OFF_ALL_CH);
#ifdef MBED
	// pc.printf("S-Voltage conversion completed\n");
#else
	// printf("S-Voltage conversion completed\n");
#endif
}

/**
 *******************************************************************************
 * @brief Read S-Voltages
 *******************************************************************************
 */
void adBms6830_read_s_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS, DCP_OFF, OW_OFF_ALL_CH);
	adBms6830_Snap();
	adBmsReadData(tIC, &ic[0], RDSVA, S_volt, A);
	adBmsReadData(tIC, &ic[0], RDSVB, S_volt, B);
	adBmsReadData(tIC, &ic[0], RDSVC, S_volt, C);
	adBmsReadData(tIC, &ic[0], RDSVD, S_volt, D);
	adBmsReadData(tIC, &ic[0], RDSVE, S_volt, E);
	adBmsReadData(tIC, &ic[0], RDSVF, S_volt, F);
	adBms6830_Unsnap();
	// printVoltages(tIC, &ic[0], S_volt);
}

/**
 *******************************************************************************
 * @brief Start Avarage Cell Voltage Measurement
 *******************************************************************************
 */
void adBms6830_start_avgcell_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
#ifdef MBED
	// pc.printf("Avg Cell voltage conversion completed\n");
#else
	// printf("Avg Cell voltage conversion completed\n");
#endif
}

/**
 *******************************************************************************
 * @brief Read Avarage Cell Voltages
 *******************************************************************************
 */
void adBms6830_read_avgcell_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBms6830_Snap();
	//HAL_Delay(1000);
	adBmsReadData(tIC, &ic[0], RDACA, AvgCell, A);
	adBmsReadData(tIC, &ic[0], RDACB, AvgCell, B);
	adBmsReadData(tIC, &ic[0], RDACC, AvgCell, C);
	adBmsReadData(tIC, &ic[0], RDACD, AvgCell, D);
	adBmsReadData(tIC, &ic[0], RDACE, AvgCell, E);
	adBmsReadData(tIC, &ic[0], RDACF, AvgCell, F);
	adBms6830_Unsnap();
	printVoltages(tIC, &ic[0], AvgCell);
}

/**
 *******************************************************************************
 * @brief Start Filtered Cell Voltages Measurement
 *******************************************************************************
 */
void adBms6830_start_fcell_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
#ifdef MBED
	// pc.printf("F Cell voltage conversion completed\n");
#else
	// printf("F Cell voltage conversion completed\n");
#endif
}

/**
 *******************************************************************************
 * @brief Read Filtered Cell Voltages
 *******************************************************************************
 */
void adBms6830_read_fcell_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBms6830_Snap();
	HAL_Delay(500);
	adBmsReadData(tIC, &ic[0], RDFCA, F_volt, A);
	adBmsReadData(tIC, &ic[0], RDFCB, F_volt, B);
	adBmsReadData(tIC, &ic[0], RDFCC, F_volt, C);
	adBmsReadData(tIC, &ic[0], RDFCD, F_volt, D);
	adBmsReadData(tIC, &ic[0], RDFCE, F_volt, E);
	adBmsReadData(tIC, &ic[0], RDFCF, F_volt, F);
	adBms6830_Unsnap();
	// printVoltages(tIC, &ic[0], F_volt);
}

/**
 *******************************************************************************
 * @brief Start AUX, VMV, V+ Voltages Measurement
 *******************************************************************************
 */
void adBms6830_start_aux_voltage_measurment(uint8_t tIC, cell_asic *ic) {
	for (uint8_t cic = 0; cic < tIC; cic++) {
		/* Init config A */
		ic[cic].tx_cfga.refon = PWR_UP;
		ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
	}
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
#ifdef MBED
	// pc.printf("Aux voltage conversion completed\n");
#else
	// printf("Aux voltage conversion completed\n");
#endif
}

/**
 *******************************************************************************
 * @brief Read AUX, VMV, V+ Voltages
 *******************************************************************************
 */
void adBms6830_read_aux_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsReadData(tIC, &ic[0], RDAUXA, Aux, A);
	adBmsReadData(tIC, &ic[0], RDAUXB, Aux, B);
	adBmsReadData(tIC, &ic[0], RDAUXC, Aux, C);
	adBmsReadData(tIC, &ic[0], RDAUXD, Aux, D);
	printVoltages(tIC, &ic[0], Aux);
}

/**
 *******************************************************************************
 * @brief Start Redundant GPIO Voltages Measurement
 *******************************************************************************
 */
void adBms6830_start_raux_voltage_measurment(uint8_t tIC, cell_asic *ic) {
	for (uint8_t cic = 0; cic < tIC; cic++) {
		/* Init config A */
		ic[cic].tx_cfga.refon = PWR_UP;
		ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
	}
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBms6830_Adax2(AUX_ALL);
#ifdef MBED
	// pc.printf("RAux voltage conversion completed\n");
#else
	// printf("RAux voltage conversion completed\n");
#endif
}

/**
 *******************************************************************************
 * @brief Read Redundant GPIO Voltages
 *******************************************************************************
 */
void adBms6830_read_raux_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBms6830_Adax2(AUX_ALL);
	adBmsReadData(tIC, &ic[0], RDRAXA, RAux, A);
	adBmsReadData(tIC, &ic[0], RDRAXB, RAux, B);
	adBmsReadData(tIC, &ic[0], RDRAXC, RAux, C);
	adBmsReadData(tIC, &ic[0], RDRAXD, RAux, D);
	printVoltages(tIC, &ic[0], RAux);
}

/**
 *******************************************************************************
 * @brief Read Status Reg. A, B, C, D and E.
 *******************************************************************************
 */
void adBms6830_read_status_registers(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, C);
	adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE,
			AUX_CH_TO_CONVERT);
	adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT,
			DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);

	adBmsReadData(tIC, &ic[0], RDSTATA, Status, A);
	adBmsReadData(tIC, &ic[0], RDSTATB, Status, B);
	adBmsReadData(tIC, &ic[0], RDSTATC, Status, C);
	adBmsReadData(tIC, &ic[0], RDSTATD, Status, D);
	adBmsReadData(tIC, &ic[0], RDSTATE, Status, E);
	// printPollAdcConvTime(pladc_count);
	// printStatus(tIC, &ic[0], Status, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Loop measurment.
 *******************************************************************************
 */
void measurement_loop() {
	if (MEASURE_CELL == ENABLED) {
		adBms6830_Snap();
		adBmsReadData(TOTAL_IC, &IC[0], RDCVA, Cell, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDCVB, Cell, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDCVC, Cell, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDCVD, Cell, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDCVE, Cell, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDCVF, Cell, F);
		adBms6830_Unsnap();
		// printVoltages(TOTAL_IC, &IC[0], Cell);
	}

	if (MEASURE_AVG_CELL == ENABLED) {
		adBms6830_Snap();
		adBmsReadData(TOTAL_IC, &IC[0], RDACA, AvgCell, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDACB, AvgCell, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDACC, AvgCell, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDACD, AvgCell, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDACE, AvgCell, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDACF, AvgCell, F);
		adBms6830_Unsnap();
		// printVoltages(TOTAL_IC, &IC[0], AvgCell);
	}

	if (MEASURE_F_CELL == ENABLED) {
		adBms6830_Snap();
		adBmsReadData(TOTAL_IC, &IC[0], RDFCA, F_volt, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCB, F_volt, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCC, F_volt, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCD, F_volt, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCE, F_volt, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCF, F_volt, F);
		adBms6830_Unsnap();
		// printVoltages(TOTAL_IC, &IC[0], F_volt);
	}

	if (MEASURE_S_VOLTAGE == ENABLED) {
		adBmsReadData(TOTAL_IC, &IC[0], RDSVA, S_volt, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVB, S_volt, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVC, S_volt, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVD, S_volt, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVE, S_volt, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVF, S_volt, F);
		// printVoltages(TOTAL_IC, &IC[0], S_volt);
	}

	if (MEASURE_AUX == ENABLED) {
		adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE,
				AUX_CH_TO_CONVERT);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXA, Aux, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXB, Aux, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXC, Aux, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXD, Aux, D);
		// printVoltages(TOTAL_IC, &IC[0], Aux);
	}

	if (MEASURE_RAUX == ENABLED) {
		adBmsWakeupIc(TOTAL_IC);
		adBms6830_Adax2(AUX_CH_TO_CONVERT);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXA, RAux, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXB, RAux, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXC, RAux, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXD, RAux, D);
		// printVoltages(TOTAL_IC, &IC[0], RAux);
	}

	if (MEASURE_STAT == ENABLED) {
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATA, Status, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATB, Status, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATC, Status, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATD, Status, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATE, Status, E);
		// printStatus(TOTAL_IC, &IC[0], Status, ALL_GRP);
	}
}

/**
 *******************************************************************************
 * @brief Clear Cell measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_cell_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRCELL);
#ifdef MBED
	// pc.printf("Cell Registers Cleared\n\n");
#else
	// printf("Cell Registers Cleared\n\n");
#endif
}

/**
 *******************************************************************************
 * @brief Clear Aux measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_aux_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRAUX);
#ifdef MBED
	// pc.printf("Aux Registers Cleared\n\n");
#else
	// printf("Aux Registers Cleared\n\n");
#endif
}

/**
 *******************************************************************************
 * @brief Clear spin measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_spin_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRSPIN);
#ifdef MBED
	// pc.printf("Spin Registers Cleared\n\n");
#else
	// printf("Spin Registers Cleared\n\n");
#endif
}

/**
 *******************************************************************************
 * @brief Clear fcell measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_fcell_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRFC);
#ifdef MBED
	// pc.printf("Fcell Registers Cleared\n\n");
#else
	// printf("Fcell Registers Cleared\n\n");
#endif
}

void balanceCells(uint8_t tIC, cell_asic *ic, PWM_DUTY duty_cycle) {
	// Reset after max duration to prevent overheating
	if (tick > 200) {
		stopBalancing(tIC, ic);
		tick = 0;
		// Consider updating target voltage after each cycle
		target_lowest_cell = lowest_cell;
		return;
	}

	// Initialize target when first called
	if (target_lowest_cell == -1) {
		target_lowest_cell = lowest_cell;
	}

	// Only update balancing configuration periodically
	if (tick == 10) {
		// Clear balance mask for new calculation
		multiMask = 0;

		for (uint8_t dev = 0; dev < tIC; ++dev) {
			// Start with all balance switches off
			ic[dev].tx_cfgb.dcc = 0;

			for (uint8_t ch = 0; ch < cell_count; ++ch) {
				float v = getVoltage(ic[dev].cell.c_codes[ch]);

				// Improved logic: Balance cells above target with a small hysteresis
				if (v > (target_lowest_cell + 0.01)) { // 10mV hysteresis
													   // Set this cell for balancing
					multiMask |= (1 << ch);
					// Configure PWM duty cycle
					ic[dev].PwmA.pwma[ch] = duty_cycle;
				} else {
					// Ensure PWM is off for cells we don't balance
					ic[dev].PwmA.pwma[ch] = PWM_0_0_PCT;
				}
			}

			// Apply the mask directly (cleaner than the previous approach)
			ic[dev].tx_cfgb.dcc = ConfigB_DccBits(multiMask, DCC_BIT_SET);
		}
	}

	// Send configuration to the hardware - this should happen every time
	// to ensure the balancing continues even if we don't update the mask
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRPWM1, Pwm, A); /* cells 1-8 */

	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRPWM2, Pwm, B); /* cells 9-16 */

	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B); /* push DCC */

	// Enable S-pin control
	adBmsWakeupIc(tIC);
	spiSendCmd(UNMUTE);
}

void stopBalancing(uint8_t tIC, cell_asic *ic) {
	// Clear all balance control
	multiMask = 0;

	for (uint8_t dev = 0; dev < tIC; ++dev) {
		// Clear all DCC bits for all cells
		ic[dev].tx_cfgb.dcc = 0;

		// Also ensure all PWM settings are zero
		for (uint8_t ch = 0; ch < cell_count; ++ch) {
			ic[dev].PwmA.pwma[ch] = PWM_0_0_PCT;
		}
	}

	// Update hardware registers
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRPWM1, Pwm, A);

	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRPWM2, Pwm, B);

	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);

	// Ensure S-pins are operational
	adBmsWakeupIc(tIC);
	spiSendCmd(UNMUTE);
}
