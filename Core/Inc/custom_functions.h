#ifndef CUSTOM_FUNCTIONS_H
#define CUSTOM_FUNCTIONS_H

#include "main.h"
#include <stdint.h>        // for int16_t
#include "adBms6830Data.h" // for cell_asic definition

/* ==== Constants ==== */
extern const float TEMP_LIMIT;
extern const float LOWER_TEMP_LIMIT;
extern const float OV_THRESHOLD;
extern const float UV_THRESHOLD;
extern const int OWC_Threshold;
extern const int OWA_Threshold;
extern const uint32_t LOOP_MEASUREMENT_COUNT;
extern const uint16_t MEASUREMENT_LOOP_TIME;
extern const uint8_t CELL_UV_FAULT;
extern const uint8_t CELL_OV_FAULT;
extern const uint8_t CELL_TEMP_FAULT;
extern const uint8_t READY_POWER;
extern const uint8_t CHARGE_POWER;
extern const uint16_t PRECHARGE_MAX_TIME;

#define NUM_CELLS_PER_IC 10
#define NUM_TEMPS_PER_IC 10

/* ==== Globals ==== */
extern uint8_t cell_count;
extern uint8_t cell_fault;
extern uint8_t temp_fault;
extern uint8_t current_sensor_fault;
extern int accy_status;
extern float current;
extern float cell_resistance;
extern float soc;
extern float lowest_cell;
extern float highest_cell;
extern float avg_cell;
extern float lowest_temp;
extern float highest_temp;
extern int lowest_temp_ID;
extern int highest_temp_ID;
extern float avg_temp;
extern uint32_t loop_count;
extern uint32_t pladc_count;
extern float fan_status;
extern uint8_t is_charging; // 1 if charging, 0 if not
extern float ccl;
extern float dcl;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* ==== Function Prototypes ==== */
int user_adBms6830_cellFault(uint8_t tIC, cell_asic *IC);
int user_adBms6830_tempFault(uint8_t tIC, cell_asic *IC);
void user_adBms6830_setFaults(void);
void user_adBms6830_getAccyStatus(void);
uint8_t getCurrentSensorData(void);
float getCurrentVoltage(int value);
void printFloat(float num);
void start_precharge_sequence(void);
float getPackVoltage(int totalIC, cell_asic *ICs);
float updateSOC(void);
void fanPWMControl(float max_temp, TIM_HandleTypeDef *htimPWM);
void populateIC(cell_asic *IC, uint8_t tIC);
void print_fault_summary(void);
float calcCCL(void);
float calcDCL(void);
float SOCtoVoltage(float soc);
#define PRINT_ON 1

#endif // CUSTOM_FUNCTIONS_H
