#include "custom_functions.h"
#include "serialPrintResult.h"
#include "adBms_Application.h"
#include <math.h>
#include <stdio.h>
#include <string.h> // for memset used in balanceCells

/* ==== Constants ==== */
const float TEMP_LIMIT = 60.0;
const float LOWER_TEMP_LIMIT = -10.0;
const float OV_THRESHOLD = 4.2;
const float UV_THRESHOLD = 2.5;
const int OWC_Threshold = 2000;
const int OWA_Threshold = 50000;
const uint32_t LOOP_MEASUREMENT_COUNT = 1;
const uint16_t MEASUREMENT_LOOP_TIME = 2;
const uint8_t CELL_UV_FAULT = 0x01;
const uint8_t CELL_OV_FAULT = 0x02;
const uint8_t CELL_TEMP_FAULT = 0x03;
const uint8_t CURRENT_SENSOR_FAULT = 0x04;
const uint8_t READY_POWER = 0x01;
const uint8_t CHARGE_POWER = 0x02;
const uint16_t PRECHARGE_MAX_TIME = 5000;
const float TEMP_LIMIT_HIGH = 55.0;
const float TEMP_LIMIT_MED = 48.0;
const float TEMP_LIMIT_LOW = 40.0;

const float BATTERY_CAPACITY_Ah = 18.0f; // As per your notes: 18 Ah
const float BATTERY_CAPACITY_COULOMBS = BATTERY_CAPACITY_Ah * 3600.0f; // Total capacity in Amp-seconds
const float REST_CURRENT_THRESHOLD = 0.5f;   // 0.5 Amps: considered 'at rest'
const uint32_t REST_DURATION_MS = 180000;

/* ==== Globals ==== */
uint8_t cell_count = 10; //per IC i think?
uint8_t cell_fault = 0;
uint8_t temp_fault = 0;
uint8_t current_sensor_fault = 0;
int accy_status = 0;
float current = 0.0;
float cell_resistance = 0.00005; // 0.05 mOhm
float soc = 0.0;
float lowest_cell = 0.0;
float highest_cell = 0.0;

float avg_cell = 0.0;
float delta_cell = 0.0;
uint8_t is_charging = 0;
float ccl = 0;
float dcl = 0;

float lowest_temp = 0.0;
float highest_temp = 0.0;
int lowest_temp_ID = 0;
int highest_temp_ID = 0;
float avg_temp = 0.0;
float delta_temp = 0.0;
#define MAX_ALLOWED_TEMP_FAULTS 10

uint32_t loop_count = 0;
uint32_t pladc_count = 0;
float fan_status = 0.0;
float initial_soc = -1.0;       // Initial SOC
volatile float coulombs = 0.0f; // Coulombs counter for energy calculation

/* ==== Timing for Current Sensor ==== */
static uint32_t last_time;
static uint32_t rest_start_time = 0;
static bool is_resting = false;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

float voltagetoSOC(float voltage);

/* ==== Fault tracking ==== */
#define MAX_FAULT_ENTRIES 20 // Maximum number of faults to track
#define FAULT_TYPE_UV 1      // Under-voltage fault
#define FAULT_TYPE_OV 2      // Over-voltage fault
#define FAULT_TYPE_TEMP 3    // Temperature fault
typedef struct {
	uint16_t cell_id;   // IC# * 10 + Cell# (e.g. 0*10+5 = 5 for IC0, Cell5)
	uint8_t fault_type; // Type of fault (UV, OV, TEMP)
	float fault_value;  // Voltage or temperature value that caused the fault
	uint32_t timestamp; // When the fault occurred (in ms)
	bool active;        // Whether the fault is still active
} cell_fault_entry_t;

cell_fault_entry_t fault_log[MAX_FAULT_ENTRIES];
uint8_t num_active_faults = 0;


/* ==== SOC vs. Voltage ==== */
const int SOC_LUT_SIZE = 9;
const float SOC_LUT_VOLTAGES[] = {2.5f, 3.15f, 3.41f, 3.53f, 3.66f, 3.77f, 3.88f, 3.98f, 4.10f};
const float SOC_LUT_PERCENT[]  = {0.0f, 12.0f, 25.0f, 37.0f, 50.0f, 62.0f, 75.0f, 87.0f, 100.0f};

/* ==== DCL vs. Temperature ==== */
const int DCL_LUT_SIZE = 8;
const float DCL_LUT_TEMP[]     = {0.0f, 5.0f, 10.0f, 15.0f, 45.0f, 50.0f, 55.0f, 60.0f};
const float DCL_LUT_CURRENT[]  = {0.0f, 40.0f, 100.0f, 180.0f, 180.0f, 70.0f, 10.0f, 0.0f};

/* ==== CCL vs. Temperature ==== */
const int CCL_LUT_SIZE = 11;
const float CCL_LUT_TEMP[]     = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 35.0f, 40.0f, 45.0f, 50.0f, 55.0f, 60.0f};
const float CCL_LUT_CURRENT[]  = {0.0f, 0.0f, 10.0f, 20.0f, 30.0f, 30.0f, 20.0f, 10.0f, 5.0f,  0.0f,  0.0f};


/* === Functions ==== */
/**
 * @brief Add a new fault to the fault log or update existing one
 *
 * @param ic_num IC number (0-based)
 * @param cell_num Cell number (0-based)
 * @param fault_type Type of fault (UV, OV, TEMP)
 * @param fault_value Value that caused the fault
 * @return index of the fault entry in the array
 */
int add_fault(uint8_t ic_num, uint8_t cell_num, uint8_t fault_type,
		float fault_value) {
	uint16_t cell_id = ic_num * 10 + cell_num;

	// First check if this fault already exists
	for (int i = 0; i < MAX_FAULT_ENTRIES; i++) {
		if (fault_log[i].active && fault_log[i].cell_id == cell_id
				&& fault_log[i].fault_type == fault_type) {
			// Update existing fault entry
			fault_log[i].fault_value = fault_value;
			return i;
		}
	}

	// Find an empty slot for a new fault
	for (int i = 0; i < MAX_FAULT_ENTRIES; i++) {
		if (!fault_log[i].active) {
			fault_log[i].cell_id = cell_id;
			fault_log[i].fault_type = fault_type;
			fault_log[i].fault_value = fault_value;
			fault_log[i].timestamp = HAL_GetTick();
			fault_log[i].active = true;
			num_active_faults++;

			// Print info about the new fault
			if (PRINT_ON) printf("New fault: IC%d Cell%d Type:%d Value:", ic_num + 1,
					cell_num + 1, fault_type);
			if (PRINT_ON) printFloat(fault_value);

			return i;
		}
	}

	// If we get here, the fault log is full - just return -1
	if (PRINT_ON) printf("Fault log full! Cannot record new faults\n");
	return -1;
}

/**
 * @brief Clear a fault that is no longer active
 *
 * @param ic_num IC number (0-based)
 * @param cell_num Cell number (0-based)
 * @param fault_type Type of fault (UV, OV, TEMP)
 */
void clear_fault(uint8_t ic_num, uint8_t cell_num, uint8_t fault_type) {
	uint16_t cell_id = ic_num * 10 + cell_num;

	for (int i = 0; i < MAX_FAULT_ENTRIES; i++) {
		if (fault_log[i].active && fault_log[i].cell_id == cell_id
				&& fault_log[i].fault_type == fault_type) {

			if (PRINT_ON) printf("Cleared fault: IC%d Cell%d Type:%d Value:", ic_num + 1,
					cell_num + 1, fault_type + 1);
			if (PRINT_ON) printFloat(fault_log[i].fault_value);

			fault_log[i].active = false;
			num_active_faults--;
			return;
		}
	}
}

/**
 * @brief Initialize the fault log
 */
void init_fault_log(void) {
	for (int i = 0; i < MAX_FAULT_ENTRIES; i++) {
		fault_log[i].active = false;
	}
	num_active_faults = 0;
}

/**
 * @brief Print a summary of all active faults in a concise format
 *
 * This function prints all active faults in a compact format showing:
 * - Cell ID (IC# * 10 + Cell#)
 * - Value that caused the fault
 * - Type of fault (V for voltage, T for temperature)
 */
void print_fault_summary(void) {
	if (PRINT_ON) {
	if (num_active_faults == 0) {
		printf("No active faults\n");
		return;
	}

	printf("Faulted cells:\n");

	int count = 0;
	for (int i = 0; i < MAX_FAULT_ENTRIES; i++) {
		if (fault_log[i].active) {
			uint8_t ic_num = fault_log[i].cell_id / 10;
			uint8_t cell_num = fault_log[i].cell_id % 10;

			// Print cell information
			printf("IC%d Cell%d: ", ic_num + 1, cell_num + 1);

			// Print fault value with appropriate units
			switch (fault_log[i].fault_type) {
			case FAULT_TYPE_UV:
			case FAULT_TYPE_OV:
				printf("%.2fV", fault_log[i].fault_value);
				break;
			case FAULT_TYPE_TEMP:
				printf("%.1fC", fault_log[i].fault_value);
				break;
			default:
				printf("%.2f?", fault_log[i].fault_value);
			}

			// Add separator or newline
			count++;
			if (count % 4 == 0) { // Four entries per line
				printf("\n");
			} else {
				printf("; ");
			}
		}
	}

	// Ensure we end with a newline
	if (count % 4 != 0) {
		printf("\n");
	}
	}
}

void populateIC(cell_asic *IC, uint8_t tIC) {
	uint32_t timingshits = HAL_GetTick();
	adBms6830_start_adc_cell_voltage_measurment(tIC);
	if (PRINT_ON) printf("time to read once: %d", HAL_GetTick() - timingshits);

	Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	timingshits = HAL_GetTick();
	adBms6830_read_cell_voltages(tIC, &IC[0]);
	if (PRINT_ON) printf("time to read once: %d", HAL_GetTick() - timingshits);

	int c_fault = user_adBms6830_cellFault(tIC, &IC[0]);
	if (c_fault != 0) {
		cell_fault = c_fault;
	}
	Delay_ms(8);
	timingshits = HAL_GetTick();
	adBms6830_start_aux_voltage_measurment(tIC, &IC[0]);
	if (PRINT_ON) printf("time to read once: %d", HAL_GetTick() - timingshits);

	Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
	timingshits = HAL_GetTick();

	adBms6830_read_aux_voltages(tIC, &IC[0]);
	if (PRINT_ON) printf("time to read once: %d", HAL_GetTick() - timingshits);

	int t_fault = user_adBms6830_tempFault(tIC, &IC[0]);
	if (t_fault == 1) {
		temp_fault = CELL_TEMP_FAULT;
	}

	timingshits = HAL_GetTick();
	// Current sensor data + fault
	uint8_t current_fault = getCurrentSensorData();
	if (current_fault != 0) {
		current_sensor_fault = CURRENT_SENSOR_FAULT;
		// printf("Current sensor fault detected\n");
	}
	if (PRINT_ON) printf("time to read once: %d", HAL_GetTick() - timingshits);
}

int user_adBms6830_cellFault(uint8_t tIC, cell_asic *IC) {
	int error = 0;
	int16_t temp;
	float voltage;
	float adjusted_voltage;

	lowest_cell = 100.0; // Initialize to a high value
	highest_cell = 0.0;  // Initialize to a low value

	for (uint8_t ic = 0; ic < tIC; ic++) {
		for (uint8_t index = 0; index < cell_count; index++) {
			temp = IC[ic].cell.c_codes[index];
			voltage = getVoltage(temp);

			// Adjust voltage based on current if current sensor is not faulted
			if (current_sensor_fault == 0) {
				if (accy_status == READY_POWER) {
					// When discharging, actual cell voltage is higher than measured
					adjusted_voltage = voltage + (current * cell_resistance);
				} else if (accy_status == CHARGE_POWER) {
					// When charging, actual cell voltage is lower than measured
					adjusted_voltage = voltage - (current * cell_resistance);
				} else {
					adjusted_voltage = voltage; // No adjustment needed
				}
			} else {
				adjusted_voltage = voltage; // No adjustment if current sensor is faulted
			}

			if (adjusted_voltage < lowest_cell) {
				lowest_cell = adjusted_voltage;
			}
			if (adjusted_voltage > highest_cell) {
				highest_cell = adjusted_voltage;
			}

			// Check for under-voltage faults using adjusted voltage
			if (adjusted_voltage < UV_THRESHOLD) {
				error = CELL_UV_FAULT;
				add_fault(ic, index, FAULT_TYPE_UV, adjusted_voltage);
			} else {
				// Clear the UV fault if it exists
				clear_fault(ic, index, FAULT_TYPE_UV);
			}

			// Check for over-voltage faults using adjusted voltage
			if (adjusted_voltage > OV_THRESHOLD) {
				error = CELL_OV_FAULT;
				add_fault(ic, index, FAULT_TYPE_OV, adjusted_voltage);
			} else {
				// Clear the OV fault if it exists
				clear_fault(ic, index, FAULT_TYPE_OV);
			}
		}
	}

	delta_cell = highest_cell - lowest_cell;
	return error;
}

int user_adBms6830_tempFault(uint8_t tIC, cell_asic *IC) {
	int error = 0;
	int16_t temp;
	float temperature;
	float V;

	lowest_temp = 1000.0; // Initialize to a high value
	highest_temp = 0.0;   // Initialize to a low value
	float temp_sum = 0.0;
	int faulted_cell_count = 0;

	for (uint8_t ic = 0; ic < tIC; ic++) {
		for (uint8_t index = 0; index < cell_count; index++) {
			if (ic * 10 + index + 1 == 55) {
				continue;
			}

			temp = IC[ic].aux.a_codes[index];
			V = getVoltage(temp);
			temperature = -225.6985 * (V * V * V) + 1310.5937 * (V * V)
																+ -2594.7697 * V + 1767.8260;

			// Check for temperature faults
			if (temperature > TEMP_LIMIT || temperature < LOWER_TEMP_LIMIT) {
				faulted_cell_count++;
				if (faulted_cell_count > MAX_ALLOWED_TEMP_FAULTS) {
					error = 1;
					add_fault(ic, index, FAULT_TYPE_TEMP, temperature);
				}
			} else {


				if (temperature < lowest_temp) {
					lowest_temp_ID = ic * 10 + index + 1;
					if (lowest_temp_ID != 11)
						lowest_temp = temperature;
				}
				if (temperature > highest_temp) {
					highest_temp_ID = ic * 10 + index + 1;
					if (highest_temp_ID != 45)
						highest_temp = temperature;
				}

				avg_temp += temperature;


				// Clear the temperature fault if it exists
				clear_fault(ic, index, FAULT_TYPE_TEMP);
			}
		}
	}

	avg_temp /= (tIC * cell_count) - faulted_cell_count;
	return error;
}

void user_adBms6830_setFaults(void) {
	if (cell_fault == CELL_UV_FAULT || cell_fault == CELL_OV_FAULT) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // TODO: Change to PB3 on next iteration

		if (accy_status == CHARGE_POWER) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // TODO: Change to PB3 on next iteration
			is_charging = 0;
		}
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // TODO: Change to PB3 on next iteration
		if (accy_status == CHARGE_POWER) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // TODO: Change to PB3 on next iteration
			is_charging = 1;
		}
	}

	if (temp_fault == CELL_TEMP_FAULT) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // TODO: Change to PB3 on next iteration

		if (accy_status == CHARGE_POWER) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // TODO: Change to PB3 on next iteration
			is_charging = 0;
		}
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // TODO: Change to PB3 on next iteration
		if (accy_status == CHARGE_POWER) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // TODO: Change to PB3 on next iteration
			is_charging = 1;
		}
	}
}

void user_adBms6830_getAccyStatus(void) {
	GPIO_PinState charge_power = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
	GPIO_PinState ready_power = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);

	if (charge_power == GPIO_PIN_SET && ready_power == GPIO_PIN_SET) {
		accy_status = -1;
	} else if (charge_power == GPIO_PIN_SET) {
		accy_status = CHARGE_POWER;
	} else if (ready_power == GPIO_PIN_SET) {
		accy_status = READY_POWER;
	} else {
		accy_status = 0;
	}
}

/**
 * @brief Get current sensor data with hysteresis to prevent jumps between ranges
 *
 * This function reads from two ADCs that measure current at different ranges,
 * then applies hysteresis logic to smoothly transition between ranges.
 */
uint8_t getCurrentSensorData(void) {
	int adc1Value = 0;
	int adc2Value = 0;

// Static variables to maintain state between calls
	static int current_range = 0;                 // 0: low range, 1: high range
	static const float LOW_TO_HIGH_THRESHOLD = 29.0; // Threshold to switch from low to high
	static const float HIGH_TO_LOW_THRESHOLD = 24.0; // Threshold to switch from high to low

// Start ADC conversions
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

// Get ADC values with timeout
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		adc1Value = HAL_ADC_GetValue(&hadc1);
	}

	if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
		adc2Value = HAL_ADC_GetValue(&hadc2);
	}

// Convert ADC values to voltages
	float v1 = getCurrentVoltage(adc1Value);
	float v2 = getCurrentVoltage(adc2Value);

// Calculate current from both ranges
	float current_high = 159.6343 * v2 - 401.4685;
	float current_low = 13.2615 * v1 - 34.3672;

	if (fabs(current_high) > 400.0) {
		return CURRENT_SENSOR_FAULT; // High current sensor out of range
	}

// Apply hysteresis logic to determine range
	float abs_low = fabs(current_low);

	if (current_range == 0 && abs_low > LOW_TO_HIGH_THRESHOLD) {
		// Switch from low range to high range when current exceeds threshold
		current_range = 1;
	} else if (current_range == 1 && abs_low < HIGH_TO_LOW_THRESHOLD) {
		// Switch from high range to low range when current drops below threshold
		current_range = 0;
	}

// Select current value based on range with appropriate sign
	if (current_range == 0) {
		// Use low range value
		if (accy_status == CHARGE_POWER) {
			current = -current_low;
		} else {
			current = current_low;
		}
	} else {
		// Use high range value
		if (accy_status == CHARGE_POWER) {
			current = -current_high;
		} else {
			current = current_high;
		}
	}
	if (PRINT_ON) {
	printf("Current Sensor Low Current: ");
	printFloat(current_low);
	printf("Current Sensor High Current: ");
	printFloat(current_high);
	printf("Selected Current: ");
	printFloat(current);
	}

	return 0; // Return 0 to indicate success
}

float getCurrentVoltage(int value) {
	return 0.001444863364 * (float) value + 0.110218620256712;
}

void printFloat(float num) {
	int intPart = (int) num;
	int fracPart = (int) (fabs(num - intPart) * 10000 + 0.5f);
	printf("%d.%04d\n", intPart, fracPart);
}

// Function to get pack voltage and update lowest, highest, and average cell voltages
float getPackVoltage(int totalIC, cell_asic *ICs) {
	float pack_voltage_sum = 0.0f;
	lowest_cell = 100.0f; // Initialize to a high value
	highest_cell = 0.0f;  // Initialize to a low value
	for (int i = 0; i < totalIC; i++) {
		for (int j = 0; j < NUM_CELLS_PER_IC; j++) {
			float cell_voltage = getVoltage(ICs[i].cell.c_codes[j]);
			pack_voltage_sum += cell_voltage;
			if (cell_voltage < lowest_cell) {
				lowest_cell = cell_voltage;
			}
			if (cell_voltage > highest_cell) {
				highest_cell = cell_voltage;
			}
		}
	}
	avg_cell = pack_voltage_sum / (totalIC * NUM_CELLS_PER_IC);

	return pack_voltage_sum;
}

#define SOC_READ_TIMEOUT 30000 // wait time before it takes a SOC read based off settled voltage, in ms
uint32_t lastNonZeroCurrentTime = 0;
uint8_t waitingForSOCReinit = 0;

float updateSOC() {
	if (cell_fault != 0) {
		soc = 0.0f;
		return soc;
	}


	soc = voltagetoSOC(avg_cell);
	if (soc > 50) {
		soc = voltagetoSOC(highest_cell);
	} else if (soc < 50) {
		soc = voltagetoSOC(lowest_cell);
	}

	uint32_t current_time = HAL_GetTick();

	if (fabs(current) < REST_CURRENT_THRESHOLD) {
		if (!is_resting) {
			is_resting = true;
			rest_start_time = current_time;
		}
	} else {
		is_resting = false;
	}

	if (is_resting && (current_time - rest_start_time > REST_DURATION_MS)) {
		soc = voltagetoSOC(avg_cell);

		// reset the coloumb counter
		coulombs = 0.0f;
		initial_soc = soc;

		rest_start_time = current_time;
	}

	// coloumb counting
	else {
		if (initial_soc < 0.0f) {
			initial_soc = voltagetoSOC(avg_cell);
			soc = initial_soc;
			last_time = current_time;
		}

		float delta_t_sec = (current_time - last_time) / 1000.0f;
		last_time = current_time;


		coulombs += current * delta_t_sec;

		float soc_change_percent = (coulombs / BATTERY_CAPACITY_COULOMBS) * 100.0f;
		soc = initial_soc - soc_change_percent;
	}

	if (soc > 100.0f) {
		soc = 100.0f;
	}
	if (soc < 0.0f) {
		soc = 0.0f;
	}
//    if (current <= 0.5 && !waitingForSOCReinit)
//    {
//        lastNonZeroCurrentTime = HAL_GetTick();
//        waitingForSOCReinit = 1;
//    }
//
//    if (current >= 0.5)
//        waitingForSOCReinit = 0;
//
//    // Initialize SOC only once when current is close to zero
//    if (initial_soc == -1.0f && lastNonZeroCurrentTime - HAL_GetTick() >= SOC_READ_TIMEOUT)
//    {
//        initial_soc = voltagetoSOC(lowest_cell);
//        last_time = HAL_GetTick();
//        waitingForSOCReinit = 1;
//    }
//
//    uint32_t current_time = HAL_GetTick();                    // Time in milliseconds
//    float delta_t_sec = (current_time - last_time) / 1000.0f; // Convert to seconds
//    last_time = current_time;
//
//    // Coulomb counting to track SOC
//    coulombs += current * delta_t_sec;
//    soc = initial_soc - 100 * (coulombs / 64800000.0f); // 64800000 is the amount of total coulombs in 18000 Ah (in percentage)
	return soc;
}

/**
 * @brief PID controlled fan speed to maintain optimal battery temperature
 *
 * This function implements a PID controller for fan speed to keep the maximum cell
 * temperature at the target of 45°C. The controller starts ramping fans at 25°C
 * and increases speed according to PID calculation. The fans are controlled via
 * an open-drain pin where LOWER PWM values result in HIGHER fan speeds.
 *
 * @param max_temp Current maximum cell temperature in Celsius
 * @param htimPWM Timer handle for PWM output
 */
void fanPWMControl(float max_temp, TIM_HandleTypeDef *htimPWM) {
// PID constants - tune these based on your system response
	static const float Kp = 0.05f;   // Proportional gain
	static const float Ki = 0.0005f; // Integral gain
	static const float Kd = 0.01f;   // Derivative gain

// PID parameters
	static float integral = 0.0f;
	static float prev_error = 0.0f;
	static uint32_t prev_time = 0;

// Temperature thresholds
	static const float MIN_TEMP = 25.0f;    // Start fans at this temperature
	static const float TARGET_TEMP = 40.0f; // Target temperature
	static const float MAX_TEMP = 55.0f;    // Maximum allowed temperature

// Get current time for delta calculation
	uint32_t current_time = HAL_GetTick();
	float delta_time =
			(prev_time == 0) ? 1.0f : (current_time - prev_time) / 1000.0f; // In seconds
	prev_time = current_time;

// Prevent integral windup by limiting delta time
	if (delta_time > 5.0f)
		delta_time = 1.0f;

// If temperature is below minimum threshold, turn off fans
	if (max_temp < MIN_TEMP) {
		__HAL_TIM_SET_COMPARE(htimPWM, TIM_CHANNEL_4, htimPWM->Init.Period); // Full OFF (max value in open drain)
		fan_status = 0.0f;
		integral = 0.0f; // Reset integral term
		prev_error = 0.0f;
		return;
	}

// Calculate error (positive error means we're above target temp)
	float error = max_temp - TARGET_TEMP;

// Compute integral with anti-windup
	integral += error * delta_time;

// Limit integral to prevent excessive buildup
	if (integral > 100.0f)
		integral = 100.0f;
	if (integral < -100.0f)
		integral = -100.0f;

// If we're below target, slowly reduce integral
	if (error < 0 && integral > 0) {
		integral *= 0.95f; // Decay integral when under target temp
	}

// Compute derivative
	float derivative =
			(delta_time > 0.0f) ? (error - prev_error) / delta_time : 0.0f;
	prev_error = error;

// Calculate PID output (0.0 = no fan, 1.0 = max fan)
	float pid_output = Kp * error + Ki * integral + Kd * derivative;

// Emergency override for high temperatures
	if (max_temp >= MAX_TEMP) {
		pid_output = 1.0f; // Maximum fan speed
	}

// Limit output range between 0.0 and 1.0
	if (pid_output < 0.0f)
		pid_output = 0.0f;
	if (pid_output > 1.0f)
		pid_output = 1.0f;

// Calculate PWM value - NOTE: Value is inverted (1.0 = no fans, 0.0 = max fans)
// Scale between 0 and Init.Period in inverted fashion
	uint32_t pwm_value = (uint32_t) (htimPWM->Init.Period * (1.0f - pid_output));

// Update PWM output
	__HAL_TIM_SET_COMPARE(htimPWM, TIM_CHANNEL_4, pwm_value);

// Store current fan status (as percentage of max speed)
	fan_status = pid_output * 100.0f;

// Debug output
	if (PRINT_ON) printf("Temp: %.1f°C, Error: %.1f, PID: %.2f, Fan: %.0f%%, PWM: %lu\r\n",
			max_temp, error, pid_output, fan_status, pwm_value);
}

float interpolate(float x, const float x_points[], const float y_points[], int num_points) {
    if (x <= x_points[0]) {
        return y_points[0];
    }

    if (x >= x_points[num_points - 1]) {
        return y_points[num_points - 1];
    }

    for (int i = 0; i < num_points - 1; i++) {
        if (x >= x_points[i] && x <= x_points[i + 1]) {
            float x1 = x_points[i];
            float y1 = y_points[i];
            float x2 = x_points[i + 1];
            float y2 = y_points[i + 1];

            if (x1 == x2) {
                return y1;
            }

            // y = y1 + (x - x1) * (slope)
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }
    }


    return y_points[num_points - 1];
}


float voltagetoSOC(float voltage) {
    return interpolate(voltage, SOC_LUT_VOLTAGES, SOC_LUT_PERCENT, SOC_LUT_SIZE);
}


float SOCtoVoltage(float soc) {
    return interpolate(soc, SOC_LUT_PERCENT, SOC_LUT_VOLTAGES, SOC_LUT_SIZE);
}


float calcDCL() {
    return interpolate(highest_temp, DCL_LUT_TEMP, DCL_LUT_CURRENT, DCL_LUT_SIZE);
}


float calcCCL() {
    return interpolate(highest_temp, CCL_LUT_TEMP, CCL_LUT_CURRENT, CCL_LUT_SIZE);
}
