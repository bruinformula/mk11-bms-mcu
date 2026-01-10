/*
 * mcuWrapper.h
 *
 * Created on: Jan 2025
 * Bridge between ADBMS6830 driver and STM32 HAL
 */

#ifndef MCUWRAPPER_H_
#define MCUWRAPPER_H_

#include <stdint.h>
#include "main.h"

/**
 * @brief Pulls the Chip Select pin LOW for the specified chain.
 * @param link: 0 for Chain 1 (PB12), 1 for Chain 2 (PA4)
 */
void adBmsCsLow(uint8_t link);

/**
 * @brief Pulls the Chip Select pin HIGH for the specified chain.
 * @param link: 0 for Chain 1 (PB12), 1 for Chain 2 (PA4)
 */
void adBmsCsHigh(uint8_t link);

/**
 * @brief Transmits and Receives data over the specified SPI bus simultaneously.
 * @param link: 0 for SPI1, 1 for SPI2
 * @param tx_Data: Pointer to data to be transmitted
 * @param rx_Data: Pointer to buffer where received data will be stored
 * @param length: Number of bytes to transfer
 */
void adBmsWriteReadSPI(uint8_t link, uint8_t *tx_Data, uint8_t *rx_Data, uint16_t length);

/**
 * @brief Delay for a specific number of milliseconds.
 */
void adBmsWaitMs(uint16_t ms);

/**
 * @brief Delay for a specific number of microseconds.
 * Required for the wake-up pulses and precise timing sequences.
 */
void adBmsWaitUs(uint16_t us);

#endif /* MCUWRAPPER_H_ */
