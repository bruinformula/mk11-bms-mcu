/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - FIXED FOR HARDWARE CONFIG
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "adbms_can_helper.h"
#include "adBms_Application.h"
#include "custom_functions.h"
#include "adBms6830Data.h"
#include "adBms6830GenericType.h"
#include "adBms6830ParseCreate.h"
#include "serialPrintResult.h"
#include "math.h"
#include "common.h"

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
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
/* BMS CONFIGURATION */
#define TOTAL_IC 1
cell_asic IC_DATA[TOTAL_IC];

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {44, 22, 44, 22, 44, 22, 44, 22};
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8] = {0};
static FDCAN_BMS_CONTEXT g_fdcan_bms_ctx;
FDCAN_BMS_CONTEXT *FDCAN_BMS_CONTEXT_INSTANCE = &g_fdcan_bms_ctx;

int8_t balancing = 0; // 0: No balancing, 1: Balancing

/** TEST MODE SELECTION **/
#define BMS_TRANSMIT_MODE 1
#define BMS_ECHO_ENABLE 0

/* FDCAN Test Debug Variables */
volatile uint32_t fdcan_rx_count = 0;
volatile uint32_t fdcan_last_rx_id = 0;
volatile uint8_t fdcan_last_rx_data[8] = {0};
volatile uint32_t fdcan_rx_error_count = 0;
volatile uint32_t fdcan_tx_count = 0;

/* FDCAN Status Registers for debugging */
volatile uint32_t fdcan_psr_register = 0;
volatile uint32_t fdcan_ecr_register = 0;
volatile uint32_t fdcan_txfqs_register = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
    BMS_STATE_INIT,
    BMS_STATE_IDLE,
    BMS_STATE_PRECHARGE,
    BMS_STATE_READY,
    BMS_STATE_CHARGING,
    BMS_STATE_FAULT
} BMS_State_t;

typedef struct {
    BMS_State_t current_state;
    uint32_t precharge_start_time;
    float inverter_voltage;
    float initial_accy_voltage;
    bool precharge_complete;
    bool shutdown_clear;
} BMS_StateManager_t;

BMS_StateManager_t BMS_State = {
    .current_state = BMS_STATE_INIT,
    .precharge_complete = false,
    .shutdown_clear = false
};

typedef struct {
    float dc_bus_voltage;      // Main DC link voltage for precharge
    float output_voltage;      // AC output voltage
    float vab_vd_voltage;      // Phase voltages
    float vbc_vq_voltage;
    uint32_t last_update_time;
    bool data_valid;
} InverterVoltages_t;

InverterVoltages_t inverter_voltages = {0};
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /* ========== FDCAN1 SETUP ========== */
  FDCAN_FilterTypeDef sFilterConfig = {0};
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x7FF;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  TxHeader.Identifier = 0x17;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;

  /* ========== ADBMS6830 STARTUP ========== */
  /*printf("\r\n--- BMS Terminal Monitor Init ---\r\n");

  adBmsWakeupIc(TOTAL_IC);
  HAL_Delay(5); // Wait for the isoSPI to stabilize
  adBms6830_init_config(TOTAL_IC, &IC_DATA[0]);*/

  uint8_t testmessage = 0xAA;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch (BMS_State.current_state) {
		  case BMS_STATE_INIT:
			  user_adBms6830_getAccyStatus();
			  BMS_InitFaultSystem();
			  BMS_State.current_state = BMS_STATE_IDLE;
			  if (PRINT_ON) printf("BMS Initialized\n");
			  break;
		  case BMS_STATE_IDLE:
			  // Check if ready power is requested
			  if (accy_status == READY_POWER && !BMS_HasActiveFaults()) {
				  BMS_State.current_state = BMS_STATE_PRECHARGE; // Will attempt precharge
				  if (PRINT_ON) printf("Ready power requested\n");
			  } else if (accy_status == CHARGE_POWER && !BMS_HasActiveFaults()) {
				  BMS_State.current_state = BMS_STATE_CHARGING;
				  HAL_GPIO_WritePin(GPIOC, Charge_Enable_Pin, GPIO_PIN_SET);
				  if (PRINT_ON) printf("Charge mode entered\n");
			  }

			  // Allow balancing in idle if configured
			  if (balancing == 1 && !BMS_HasActiveFaults()) {
				  balanceCells(TOTAL_IC, IC, PWM_100_0_PCT);
			  }
			  break;

		  case BMS_STATE_PRECHARGE:
				if (BMS_HasActiveFaults()) {
					if (PRINT_ON) printf("Fault detected in PRECHARGE state\n");
					BMS_State.current_state = BMS_STATE_FAULT;
					break;
				}

				if (BMS_State.inverter_voltage > 10.0 || current > 0. || calcDCL() <= 0 || accy_status != READY_POWER) {
					if (PRINT_ON) printf("One or more PRECHARGE checks failed\n");
					BMS_State.current_state = BMS_STATE_FAULT;
				}


				bool successful = BMS_ExecutePrecharge();
				if (!successful) {
				  BMS_State.current_state = BMS_STATE_FAULT;
				} else {
				  BMS_State.current_state = BMS_STATE_IDLE;
				}
				populate_CAN4(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b3, &IC[0], TOTAL_IC, successful);
				break;

		  case BMS_STATE_READY:
			  // Normal driving mode
			  if (BMS_HasActiveFaults()) {
				  if (PRINT_ON) printf("Fault detected in READY state\n");
				  BMS_State.current_state = BMS_STATE_FAULT;
				  break;
			  }

			  // Control outputs
			  user_adBms6830_setFaults();
			  fanPWMControl(highest_temp, &htim1);

			  // Populate and send CAN messages
			  populate_CAN1(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b0, &IC[0], TOTAL_IC);
			  populate_CAN2(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b1, &IC[0], TOTAL_IC);
			  populate_CAN3(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b2, &IC[0], TOTAL_IC);
			  FDCAN_BMS_Mailman(&hfdcan1, FDCAN_BMS_CONTEXT_INSTANCE, HAL_GetTick(), 0);

			  // Check for mode change
			  if (accy_status != READY_POWER) {
				  HAL_GPIO_WritePin(GPIOC, POS_AIR_GND_Pin, GPIO_PIN_RESET);
				  BMS_State.current_state = BMS_STATE_IDLE;
			  }
			  break;

		  case BMS_STATE_CHARGING:
			  if (BMS_HasActiveFaults()) {
				  if (PRINT_ON) printf("Fault detected in CHARGING state\n");
				  HAL_GPIO_WritePin(GPIOC, Charge_Enable_Pin, GPIO_PIN_RESET);
				  is_charging = 0;
				  BMS_State.current_state = BMS_STATE_FAULT;
				  break;
			  }

			  user_adBms6830_setFaults();

			  // Populate and send CAN messages including charger control
			  populate_CAN1(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b0, &IC[0], TOTAL_IC);
			  populate_CAN2(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b1, &IC[0], TOTAL_IC);
			  populate_CAN3(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b2, &IC[0], TOTAL_IC);
			  populate_charge_CAN(&FDCAN_BMS_CONTEXT_INSTANCE->CAN_CHGCONTEXT, &IC[0], TOTAL_IC);
			  FDCAN_BMS_Mailman(&hfdcan1, FDCAN_BMS_CONTEXT_INSTANCE, HAL_GetTick(), is_charging);

			  // Check for mode change
			  if (accy_status != CHARGE_POWER) {
				  HAL_GPIO_WritePin(GPIOC, Charge_Enable_Pin, GPIO_PIN_RESET);
				  is_charging = 0;
				  BMS_State.current_state = BMS_STATE_IDLE;
			  }
			  break;

		  case BMS_STATE_FAULT:
			  // Ensure all outputs are safe
			  HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, POS_AIR_GND_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, NEG_AIR_GND_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, Charge_Enable_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, Discharge_Enable_Pin, GPIO_PIN_RESET);

			  user_adBms6830_setFaults();
			  stopBalancing(TOTAL_IC, IC);

			  // Continue monitoring and reporting
			  user_adBms6830_setFaults();
			  populate_CAN1(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b0, &IC[0], TOTAL_IC);
			  populate_CAN2(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b1, &IC[0], TOTAL_IC);
			  populate_CAN3(&FDCAN_BMS_CONTEXT_INSTANCE->msg_6b2, &IC[0], TOTAL_IC);
			  FDCAN_BMS_Mailman(&hfdcan1, FDCAN_BMS_CONTEXT_INSTANCE, HAL_GetTick(), 0);

			  // Check if faults cleared - require manual reset
			  if (!BMS_HasActiveFaults()) {
				  // Still stay in fault until system reset or explicit clear
				  if (PRINT_ON) printf("Faults cleared but manual reset required\n");
			  }
			  break;
	      }
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  //HAL_Delay(100);
	  HAL_SPI_Transmit(&hspi2, &testmessage, 1, 100);
	 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_Delay(100);
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 20;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6554;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Charge_Enable_Pin|Discharge_Enable_Pin|POS_AIR_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Temp_Fault_Pin|Precharge_Enable_Pin|NEG_AIR_GND_Pin|CSB1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSB_2_GPIO_Port, CSB_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W2_GPIO_Port, W2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SDC_IN_Pin */
  GPIO_InitStruct.Pin = SDC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDC_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Charge_Enable_Pin Discharge_Enable_Pin POS_AIR_GND_Pin CSB_2_Pin */
  GPIO_InitStruct.Pin = Charge_Enable_Pin|Discharge_Enable_Pin|POS_AIR_GND_Pin|CSB_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Charge_Power_Pin Ready_Power_Pin Always_On_Power_Pin */
  GPIO_InitStruct.Pin = Charge_Power_Pin|Ready_Power_Pin|Always_On_Power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Temp_Fault_Pin Precharge_Enable_Pin NEG_AIR_GND_Pin W2_Pin
                           CSB1_Pin */
  GPIO_InitStruct.Pin = Temp_Fault_Pin|Precharge_Enable_Pin|NEG_AIR_GND_Pin|W2_Pin
                          |CSB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : W1_Pin I2_Pin M1_Pin */
  GPIO_InitStruct.Pin = W1_Pin|I2_Pin|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_Pin */
  GPIO_InitStruct.Pin = M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//TODO: implement
bool BMS_ExecutePrecharge() {
	HAL_GPIO_WritePin(GPIOC, Precharge_Enable_Pin, GPIO_PIN_SET);
	float initPackVoltage = getPackVoltage(TOTAL_IC, &IC[0]);
	uint32_t start = HAL_GetTick();
	float RC = 1.0f; // TODO: actual value
	float expCurve = 0.0f;

	float prevInvVoltage = 0.0f;
	const uint32_t timeout_ms = 5000;
	const float slope_thresh_v = 0.1f;
	const float target_delta_v = 10.0f;
	uint32_t last_change = start;
	const uint32_t stable_ms = 150;
	uint32_t stable_start = 0;

	while ((HAL_GetTick() - start) < timeout_ms) {
		const float packV = getPackVoltage(TOTAL_IC, &IC[0]);
		const float invV  = BMS_State.inverter_voltage;

		// checks that we are within voltage delta and that we are stable (no more change in voltage)
		if (fabsf(packV - invV) < target_delta_v) {
		    const float dv = fabsf(invV - prevInvVoltage);

		    if (dv < slope_thresh_v) {
		        if (stable_start == 0) {
		            stable_start = HAL_GetTick();
		        }
		        if ((HAL_GetTick() - stable_start) >= stable_ms) {
		            break;
		        }
		    } else {
		        stable_start = 0;
		    }
		} else {
		    stable_start = 0;
		}

		if (fabsf(invV - prevInvVoltage) > slope_thresh_v) {
			prevInvVoltage = invV;
			last_change = HAL_GetTick();
		} else if ((HAL_GetTick() - last_change) > 200) {
			// Not moving for 200ms -> fail precharge
			HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_RESET);
			return false;
		}

		const float time_s = (float)((HAL_GetTick() - start) * 0.001f);
		expCurve = initPackVoltage * (1.0f - expf(-(time_s / RC)));
		if (expCurve > 1.0f) {
			if ((fabsf(expCurve - invV) / expCurve) > 0.10f) {
				HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_RESET);
				return false;
			}
		}

		HAL_Delay(10);
	}
	if ((HAL_GetTick() - start) >= timeout_ms) {
		HAL_GPIO_WritePin(GPIOB, Precharge_Enable_Pin, GPIO_PIN_RESET);
		return false;
	}

	HAL_GPIO_WritePin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Precharge_Enable_Pin, GPIO_PIN_RESET);

	BMS_ApplyTempCurrentLimits();
	return true;
}

//TODO: implement
bool BMS_PrechargeAbort() {
	return true;
}
/**
  * @brief  FDCAN RX FIFO 0 callback - called automatically when message arrives
  * @param  hfdcan pointer to FDCAN handle
  * @param  RxFifo0ITs interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8] = {0};

        // Read message from FIFO0
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // Update debug counters
            fdcan_rx_count++;
            fdcan_last_rx_id = RxHeader.Identifier;
            memcpy((void*)fdcan_last_rx_data, RxData, 8);

            // Process based on CAN ID
            switch (RxHeader.Identifier)
            {
                case 0xA7:  // Inverter_Voltage_Info ID
                {

                	// INV_DC_Bus_Voltage: bits 0-15 (bytes 0-1)
                	int16_t dc_bus_raw = (int16_t)((RxData[1] << 8) | RxData[0]);
                	inverter_voltages.dc_bus_voltage = dc_bus_raw * 0.1f;

                	// INV_Output_Voltage: bits 16-31 (bytes 2-3)
                	int16_t output_raw = (int16_t)((RxData[3] << 8) | RxData[2]);
                	inverter_voltages.output_voltage = output_raw * 0.1f;

                	// INV_VAB_Vd_Voltage: bits 32-47 (bytes 4-5)
                	int16_t vab_raw = (int16_t)((RxData[5] << 8) | RxData[4]);
                	inverter_voltages.vab_vd_voltage = vab_raw * 0.1f;

                	// INV_VBC_Vq_Voltage: bits 48-63 (bytes 6-7)
                	int16_t vbc_raw = (int16_t)((RxData[7] << 8) | RxData[6]);
                	inverter_voltages.vbc_vq_voltage = vbc_raw * 0.1f;

                	// Mark as valid and update timestamp
                	inverter_voltages.data_valid = true;
                	inverter_voltages.last_update_time = HAL_GetTick();
                	BMS_State.inverter_voltage = dc_bus_raw * 0.1;
                }


                default:
//                    printf("Unknown CAN ID: 0x%03lX\n", RxHeader.Identifier);
                    break;
            }
        }
        else
        {
            fdcan_rx_error_count++;
        }
    }

    // Handle FIFO0 full condition
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0)
    {
//        printf("WARNING: FIFO0 full!\n");
    }

    // Handle message lost (overflow)
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
    {
//        printf("ERROR: CAN messages lost!\n");
        fdcan_rx_error_count++;
    }
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

  /* FSAE SAFETY: Force AIRs open if the code crashes */
  HAL_GPIO_WritePin(GPIOC, POS_AIR_GND_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, NEG_AIR_GND_Pin, GPIO_PIN_RESET);

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
