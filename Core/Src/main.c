/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adBms_Application.h"
#include "thermistor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t HeaderTxBuffer[] =
		"****SPI - Two Boards communication based on Polling **** SPI Message ******** SPI Message ******** SPI Message ****";


#define VOLTAGE_DIVIDER_SCALE (3.3/5.0)
#define OFFSET_VOLTAGE  (2.5*VOLTAGE_DIVIDER_SCALE)
#define CURRENT_SENSOR_LOW_SENS ((66.7*VOLTAGE_DIVIDER_SCALE)/1000)
#define CURRENT_SENSOR_HIGH_SENS ((5.7*VOLTAGE_DIVIDER_SCALE)/1000)

uint32_t current_sensor_adc[1];
uint16_t current_sensor_low_adc;
uint16_t current_sensor_high_adc;
float current_sensor_low_voltage;
float current_sensor_high_voltage;
float current_sensor_low_amps;
float current_sensor_high_amps;

int current_range = 0;
#define LOW_TO_HIGH_THRESHOLD 29.0
#define HIGH_TO_LOW_THRESHOLD 24.0


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#endif /* __ICCARM__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FDCAN_TxHeaderTypeDef tx_msg;
uint8_t txData[8];

float voltage_conversions[10];

void computeAllVoltages(uint8_t tIC, cell_asic *ic) {
	adBms6830_read_cell_voltages(TOTAL_IC, IC);
	for (size_t i = 0; i < 10; ++i) {
		voltage_conversions[i] = getVoltage(ic->cell.c_codes[i]);
	}
}

// TODO: NEEDS CALIBRATION
float getCurrentVoltage(int value) {
	return 0.001444863364 * (float) value + 0.110218620256712;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	current_sensor_low_adc =  current_sensor_adc[0] & 0xFFFF;
	current_sensor_high_adc = (current_sensor_adc[0] >> 16) & 0xFFFF;

	current_sensor_low_voltage = getCurrentVoltage(current_sensor_low_adc);
	current_sensor_high_voltage = getCurrentVoltage(current_sensor_high_adc);

	current_sensor_low_amps = 13.2615 * current_sensor_low_voltage - 34.3672;
	current_sensor_high_amps = 159.6343 * current_sensor_high_voltage - 401.4685;

//	current_sensor_low_voltage = (current_sensor_low_adc/4095.0)*3.3;
//	current_sensor_high_voltage = (current_sensor_high_adc/4095.0)*3.3;
//
//	current_sensor_low_amps = (current_sensor_low_voltage - OFFSET_VOLTAGE)/CURRENT_SENSOR_LOW_SENS;
//	current_sensor_high_amps = (current_sensor_high_voltage - OFFSET_VOLTAGE)/CURRENT_SENSOR_HIGH_SENS;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_FDCAN1_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  FDCAN_FilterTypeDef sStdFilter= {0};
  sStdFilter.IdType = FDCAN_STANDARD_ID;
  sStdFilter.FilterIndex = 0;
  sStdFilter.FilterType = FDCAN_FILTER_RANGE;
  sStdFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sStdFilter.FilterID1 = 0x000;
  sStdFilter.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sStdFilter) != HAL_OK) {
	  Error_Handler();
  }

  FDCAN_FilterTypeDef sExtFilter = {0};
  sExtFilter.IdType = FDCAN_EXTENDED_ID;
  sExtFilter.FilterIndex = 0;
  sExtFilter.FilterType = FDCAN_FILTER_RANGE;
  sExtFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sExtFilter.FilterID1 = 0x00000000;
  sExtFilter.FilterID2 = 0x1FFFFFFF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sExtFilter) != HAL_OK) {
	  Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1)!= HAL_OK) {
	  Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
	  Error_Handler();
  }

  tx_msg.Identifier = 0x1806E5F4;
  tx_msg.IdType = FDCAN_EXTENDED_ID;
  tx_msg.TxFrameType = FDCAN_DATA_FRAME;
  tx_msg.DataLength = FDCAN_DLC_BYTES_8;
  tx_msg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_msg.BitRateSwitch = FDCAN_BRS_OFF;
  tx_msg.FDFormat = FDCAN_CLASSIC_CAN;
  tx_msg.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  tx_msg.MessageMarker = 0;

  txData[0] = 0x01; // 50 V --> 50*10 = 500 in Hex
  txData[1] = 0xF4;
  txData[2] = 0x00; // 20 A --> 20*10 = 200 in Hex
  txData[3] = 0xC8;
  txData[4] = 0; // CONTROL, 0 to START CHARGING
  txData[5] = 0;
  txData[6] = 0;
  txData[7] = 0;

//  setvbuf(stdin, NULL, _IONBF, 0);
//  adbms_main();

  // MUST ENTER PRECHARGE SEQUENCE --> DRIVE LOOP, OR BALANCE/CHARGE SEQUENCE --> TERMINATE

//  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
//  HAL_ADCEx_MultiModeStart_DMA(&hadc1, current_sensor_adc, 1);

//  adBms6830_init_config(TOTAL_IC, IC);
//  adBms6830_start_adc_cell_voltage_measurment(TOTAL_IC);
//  adBms6830_start_aux_voltage_measurment(TOTAL_IC, IC);
//  Delay_ms(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
//		computeAllVoltages(TOTAL_IC, IC);
//		computeAllTemps(TOTAL_IC, IC);
//		Delay_ms(500);

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_msg, txData);
		HAL_Delay(1000);

	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library __write function to the IAR function iar_fputc.
 * @param  file: file descriptor.
 * @param  ptr: pointer to the buffer where the data is stored.
 * @param  len: length of the data to write in bytes.
 * @retval length of the written data in bytes.
 */
#if defined(__ICCARM__)
size_t __write(int file, unsigned char const *ptr, size_t len)
{
	size_t idx;
	unsigned char const *pdata = ptr;

	for (idx = 0; idx < len; idx++)
	{
		iar_fputc((int)*pdata);
		pdata++;
	}
	return len;
}
#endif /* __ICCARM__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the LPUART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&hlpuart1);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
