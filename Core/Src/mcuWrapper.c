#include "main.h"
#include "mcuWrapper.h" // Ensure this matches your driver's header name

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

/* Pull Chip Select Low */
void adBmsCsLow(uint8_t link) {
    if (link == 0) {
        HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_RESET);
    }
}

/* Pull Chip Select High */
void adBmsCsHigh(uint8_t link) {
    if (link == 0) {
        HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_SET);
    }
}

/* Full Duplex SPI Transfer */
void adBmsWriteReadSPI(uint8_t link, uint8_t *tx_Data, uint8_t *rx_Data, uint16_t length) {
    if (link == 0) {
        HAL_SPI_TransmitReceive(&hspi2, tx_Data, rx_Data, length, 100);
    } else {
        HAL_SPI_TransmitReceive(&hspi3, tx_Data, rx_Data, length, 100);
    }
}

/* Microsecond delay for wake-up pulses */
void adBmsWaitUs(uint16_t us) {
    // Basic microsecond delay for G474 at 170MHz
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    uint32_t start = SysTick->VAL;
    while ((start - SysTick->VAL) < ticks);
}

void adBmsWaitMs(uint16_t ms) {
    HAL_Delay(ms);
}
