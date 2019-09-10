/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "message.h"
#include "clocklines.h"
#include "currentsensors.h"
#include "uartcmd.h"
#include "si4703.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern uint8_t uartDataFrame[32];
extern uint16_t clockLines_pulseWidth;
extern uint8_t clockLines_isCountersEmpty;
extern ClockLineCurrentSensor clockLineCurrentSensor;
extern uint8_t CLOCKLINES_TOTAL;
SemaphoreHandle_t uartSemaphoreHandle;
SemaphoreHandle_t lineSensorSemaphoreHandle;
SemaphoreHandle_t uvloSemaphoreHandle;
SemaphoreHandle_t Si4703SemaphoreHandle;
portBASE_TYPE xHigherPriorityTaskWoken;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;

osThreadId linesControlHandle;
osThreadId lineSensorTaskHandle;
osThreadId uartTaskHandle;
osThreadId uvloTaskHandle;
osThreadId si4703_TaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CRC_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
void StartLinesControl(void const * argument);
void StartLineSensorTask(void const * argument);
void StartUartTask(void const * argument);
void startUVLOTask(void const * argument);
void StartSi4703_Task(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	HAL_UART_Receive_IT(&huart4, uartDataFrame, sizeof(uartDataFrame));
	xSemaphoreGiveFromISR(uartSemaphoreHandle, &xHigherPriorityTaskWoken );
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance==ADC1)
	{
		static uint8_t count=0;
		static portBASE_TYPE xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		if (count==5){
			count=0;
		}
		count++;
		//HAL_ADC_Start_DMA(hadc, (uint32_t*) clockLineCurrentSensor.adc_data, clockLineCurrentSensor.totalLines);
		xSemaphoreGiveFromISR(lineSensorSemaphoreHandle, &xHigherPriorityTaskWoken );
	}

}
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance==ADC2){
		ADC2->CR1&=~ADC_CR1_AWDEN;												//disable AWD
		xSemaphoreGiveFromISR(uvloSemaphoreHandle, &xHigherPriorityTaskWoken);
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==Si4703_GPIO2_Pin)
	{
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		xSemaphoreGiveFromISR(Si4703SemaphoreHandle, &xHigherPriorityTaskWoken);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CLOCKLINES_TOTAL=4;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CRC_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  notShutDownRPi_GPIO_Port->BSRR |= notShutDownRPi_Pin<<16;
  HAL_UART_Receive_IT(&huart4, uartDataFrame, sizeof(uartDataFrame));

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  uartSemaphoreHandle = xSemaphoreCreateBinary();
  lineSensorSemaphoreHandle = xSemaphoreCreateBinary();
  uvloSemaphoreHandle = xSemaphoreCreateBinary();
  Si4703SemaphoreHandle = xSemaphoreCreateBinary();
  xHigherPriorityTaskWoken = pdFALSE;
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */


	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of linesControl */
  osThreadDef(linesControl, StartLinesControl, osPriorityNormal, 0, 1024);
  linesControlHandle = osThreadCreate(osThread(linesControl), NULL);

  /* definition and creation of lineSensorTask */
  osThreadDef(lineSensorTask, StartLineSensorTask, osPriorityNormal, 0, 256);
  lineSensorTaskHandle = osThreadCreate(osThread(lineSensorTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of uvloTask */
  osThreadDef(uvloTask, startUVLOTask, osPriorityNormal, 0, 128);
  uvloTaskHandle = osThreadCreate(osThread(uvloTask), NULL);

  /* definition and creation of si4703_Task */
  osThreadDef(si4703_Task, StartSi4703_Task, osPriorityNormal, 0, 128);
  si4703_TaskHandle = osThreadCreate(osThread(si4703_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 12;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the analog watchdog 
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 4095;
  AnalogWDGConfig.LowThreshold = 3000;
  AnalogWDGConfig.Channel = ADC_CHANNEL_13;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */
  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  hi2c1.Instance->CR2 |= I2C_CR2_LAST;
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 69;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim8.Init.Prescaler = 335;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 62449;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  Si4703_I2C_Init();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT2_pos_Pin|OUT3_pos_Pin|OUT4_pos_Pin|OUT5_pos_Pin 
                          |OUT6_pos_Pin|OUT7_pos_Pin|OUT8_pos_Pin|OUT9_pos_Pin 
                          |OUT10_pos_Pin|OUT11_pos_Pin|OUT0_pos_Pin|OUT1_pos_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUT8_neg_Pin|OUT9_neg_Pin|OUT10_neg_Pin|OUT11_neg_Pin 
                          |RELAY0_Pin|RELAY1_Pin|RELAY2_Pin|RELAY3_Pin 
                          |OUT0_neg_Pin|OUT1_neg_Pin|OUT2_neg_Pin|OUT3_neg_Pin 
                          |OUT4_neg_Pin|OUT5_neg_Pin|OUT6_neg_Pin|OUT7_neg_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Si4703_nReset_Pin|Si4703_nSen_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Si4703_GPIO1_GPIO_Port, Si4703_GPIO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, wakeUpRPi_Pin|notShutDownRPi_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUT2_pos_Pin OUT3_pos_Pin OUT4_pos_Pin OUT5_pos_Pin 
                           OUT6_pos_Pin OUT7_pos_Pin OUT8_pos_Pin OUT9_pos_Pin 
                           OUT10_pos_Pin OUT11_pos_Pin OUT0_pos_Pin OUT1_pos_Pin */
  GPIO_InitStruct.Pin = OUT2_pos_Pin|OUT3_pos_Pin|OUT4_pos_Pin|OUT5_pos_Pin 
                          |OUT6_pos_Pin|OUT7_pos_Pin|OUT8_pos_Pin|OUT9_pos_Pin 
                          |OUT10_pos_Pin|OUT11_pos_Pin|OUT0_pos_Pin|OUT1_pos_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT8_neg_Pin OUT9_neg_Pin OUT10_neg_Pin OUT11_neg_Pin 
                           RELAY0_Pin RELAY1_Pin RELAY2_Pin RELAY3_Pin 
                           OUT0_neg_Pin OUT1_neg_Pin OUT2_neg_Pin OUT3_neg_Pin 
                           OUT4_neg_Pin OUT5_neg_Pin OUT6_neg_Pin OUT7_neg_Pin */
  GPIO_InitStruct.Pin = OUT8_neg_Pin|OUT9_neg_Pin|OUT10_neg_Pin|OUT11_neg_Pin 
                          |RELAY0_Pin|RELAY1_Pin|RELAY2_Pin|RELAY3_Pin 
                          |OUT0_neg_Pin|OUT1_neg_Pin|OUT2_neg_Pin|OUT3_neg_Pin 
                          |OUT4_neg_Pin|OUT5_neg_Pin|OUT6_neg_Pin|OUT7_neg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Si4703_nReset_Pin Si4703_nSen_Pin Si4703_GPIO1_Pin */
  GPIO_InitStruct.Pin = Si4703_nReset_Pin|Si4703_nSen_Pin|Si4703_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Si4703_GPIO2_Pin */
  GPIO_InitStruct.Pin = Si4703_GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Si4703_GPIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : wakeUpRPi_Pin notShutDownRPi_Pin */
  GPIO_InitStruct.Pin = wakeUpRPi_Pin|notShutDownRPi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLinesControl */
/**
  * @brief  Function implementing the linesControl thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartLinesControl */
void StartLinesControl(void const * argument)
{
    
    
    
    
    
    
    

  /* USER CODE BEGIN 5 */
	int8_t linesId[12];
	resetLinesId(linesId);
	/* Infinite loop */
	for (;;)
	{
		if (!clockLines_isCountersEmpty){
			for (uint8_t id=0,j=0; id<12; id++)
			{
				if (clockLines[id].counter && clockLines[id].isOn)
				{
					clockLines[id].polarity=!clockLines[id].polarity;
					linesId[j]=(int8_t)id;
					j++;
				}
			}
			if(linesId[0]>=0)
			{
				//start pulse
				sendPulse(clockLines,linesId,CLOCKLINES_TOTAL, &LinesGPIO);
				startClockLinesADC(&hadc1, &clockLineCurrentSensor, linesId);
				sendCountersToUART(&huart4, clockLines, linesId);
				sendPolarityToUART(&huart4, clockLines, linesId);
				osDelay(clockLines_pulseWidth);
				//stop pulse
				stopClockLinesADC(&hadc1);
				stopPulse(&LinesGPIO);
				sendCurrentSensorsToUART(&huart4, clockLineCurrentSensor.filteredData, RESP_ADC_LINES, linesId);
				resetClockLineCurrentSensor(&clockLineCurrentSensor);
				osDelay(DEATH_TIME);
				resetLinesId(linesId);
			}
			clockLines_isCountersEmpty=1;
			for (uint8_t id=0; id<12; id++)
			{
				if (clockLines[id].counter)
				{
					clockLines[id].counter--;
					clockLines_isCountersEmpty=0;
				}
			}
		}

	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartLineSensorTask */
/**
* @brief Function implementing the lineSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLineSensorTask */
void StartLineSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartLineSensorTask */

  /* Infinite loop */
  for(;;)
  {
	  xSemaphoreTake(lineSensorSemaphoreHandle,portMAX_DELAY);
	  filter(clockLineCurrentSensor.adc_data, clockLineCurrentSensor.totalLines, 0.01, clockLineCurrentSensor.filteredData);

  }
  /* USER CODE END StartLineSensorTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */
  /* Infinite loop */
	for(;;)
	{
		if (xSemaphoreTake(uartSemaphoreHandle,portMAX_DELAY)==pdTRUE)
		{
			uartCmdParse(uartDataFrame);
		}
	}
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_startUVLOTask */

/**
* @brief Function implementing the uvloTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startUVLOTask */
void startUVLOTask(void const * argument)
{
  /* USER CODE BEGIN startUVLOTask */

	//HAL_ADC_Start(&hadc2);
	//HAL_TIM_Base_Start(&htim8);
	static float adcdata=0;
  /* Infinite loop */
  for(;;)
  {
	  xSemaphoreTake(uvloSemaphoreHandle,portMAX_DELAY);
	  if (adcdata==0) adcdata=ADC2->DR;									//Initialization ADC filtered buffer
	  filter((uint16_t*)&ADC2->DR, 1, 0.001, &adcdata);					//Anti-noise EMA filter with coefficient 0.001 (todo: tune coefficient)
      if (adcdata>=ADC2->HTR){											//Compare filtered value with high threshold. Voltage restored?
		  osDelay(1000);												//Wait for voltage stabilization
		  if (adcdata < ADC2->HTR)										//Debouncing. If voltage still low do nothing (todo: try to remove this part due to the presence of a filter)
		  {
			  continue;
		  }
		  notShutDownRPi_GPIO_Port->BSRR|= notShutDownRPi_Pin; 			//Reset NotShutdown to 1.
		  wakeUpRPi_GPIO_Port->BSRR |= wakeUpRPi_Pin;					//Set wakeUp to 1
		  osDelay(1000);
		  wakeUpRPi_GPIO_Port->BSRR |= (wakeUpRPi_Pin << 16);			//Reset wakeUp to 0
		  ADC2->HTR = 4095;												//Restore defaults for Analog WatchDog
		  ADC2->LTR = 3000;
	  }
	  else if (adcdata<ADC2->LTR)											//If voltage low
	  {
		  notShutDownRPi_GPIO_Port->BSRR |= (notShutDownRPi_Pin<<16) ;		//Call shutdown RPi
		  if (ADC2->LTR>500){												//Shift down AWD window
			  ADC2->HTR = ADC2->LTR;
			  ADC2->LTR = ADC2->LTR-500;
		  }

	  }
	  ADC2->CR1|=ADC_CR1_AWDEN;											//Enable AWD. (was disabled in callback)
  }
  /* USER CODE END startUVLOTask */
}

/* USER CODE BEGIN Header_StartSi4703_Task */
/**
* @brief Function implementing the si4703_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSi4703_Task */
void StartSi4703_Task(void const * argument)
{
  /* USER CODE BEGIN StartSi4703_Task */

  /* Infinite loop */
	//HAL_I2C_Master_Transmit(&hi2c1, Si4703_ADDR, &addr, 0, 500);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	Si4703_Init();
	Si4703_SetChannel(948);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  for(;;)
  {
	  xSemaphoreTake(Si4703SemaphoreHandle,portMAX_DELAY);
	  Si4703_Read(Si4703_REGs);
	  Si4703_SendToUART(&huart4, Si4703_REGs);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  /* USER CODE END StartSi4703_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
