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

/* ==== Fault Categories ==== */
typedef enum {
    FAULT_CATEGORY_VOLTAGE = 0,
    FAULT_CATEGORY_TEMP = 1,
    FAULT_CATEGORY_CURRENT = 2,
    FAULT_CATEGORY_COMMUNICATION = 3,
    FAULT_CATEGORY_HARDWARE = 4,
    FAULT_CATEGORY_COUNT
} FaultCategory_t;

typedef enum {
    FAULT_SEVERITY_WARNING = 0,
    FAULT_SEVERITY_ERROR = 1,
    FAULT_SEVERITY_CRITICAL = 2
} FaultSeverity_t;




/* ==== Function Prototypes ==== */
void BMS_InitFaultSystem(void);
void BMS_SetCellFault(uint8_t ic_num, uint8_t cell_num, uint8_t fault_type, FaultSeverity_t severity);
void BMS_ClearCellFault(uint8_t ic_num, uint8_t cell_num, uint8_t fault_type);
bool BMS_HasActiveFaults(void);
uint8_t BMS_GetFaultCount(FaultCategory_t category);
void print_fault_summary(void);

int user_adBms6830_cellVoltageFaults(uint8_t tIC, cell_asic *IC);
int user_adBms6830_tempFault(uint8_t tIC, cell_asic *IC);
int user_adBms6830_commFault(uint8_t tIC, cell_asic *IC);
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
float interpolate(float x, const float x_points[], const float y_points[], int num_points);
float voltagetoSOC(float voltage);
float SOCtoVoltage(float soc);


#define PRINT_ON 1

#endif // CUSTOM_FUNCTIONS_H
