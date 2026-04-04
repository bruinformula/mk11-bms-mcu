/*
 * balancing.c
 *
 *  Created on: Mar 9, 2026
 *      Author: ronitbarman
 */


#include "balancing.h"

/* Extern references to command arrays defined in adBms6830CmdList.h (included only in adBms_Application.c) */
extern uint8_t WRPWM1[2];
extern uint8_t WRPWM2[2];
extern uint8_t WRCFGB[2];
extern uint8_t UNMUTE[2];

uint16_t multiMask = 0;
float target_lowest_cell = -1;

void balanceCells(uint8_t tIC, cell_asic *ic, PWM_DUTY duty_cycle, bool update_now) {
	if (update_now) {
		// Initialize target when first called
		if (target_lowest_cell == -1) {
			target_lowest_cell = lowest_cell_voltage;
		}

		// Clear balance mask for new calculation
		multiMask = 0;

		for (uint8_t dev = 0; dev < tIC; ++dev) {
			// Start with all balance switches off
			ic[dev].tx_cfgb.dcc = 0;

			for (uint8_t ch = 0; ch < CELLS_PER_IC; ++ch) {
				float v = getVoltage(ic[dev].cell.c_codes[ch]);

				// Improved logic: Balance cells above target with a small hysteresis
				if (v > (target_lowest_cell + 0.01)) { // 10mV hysteresis
//					multiMask |= (1 << ch);
					// Configure PWM duty cycle
					ic[dev].PwmA.pwma[ch] = duty_cycle;
				} else {
					// Ensure PWM is off for cells we don't balance
					ic[dev].PwmA.pwma[ch] = PWM_0_0_PCT;
				}
			}

//			ic[dev].tx_cfgb.dcc = ConfigB_DccBits(multiMask, DCC_BIT_SET);
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
		for (uint8_t ch = 0; ch < CELLS_PER_IC; ++ch) {
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
