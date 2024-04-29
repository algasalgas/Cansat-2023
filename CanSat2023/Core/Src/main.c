/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "mmc5883.h"
#include "lora_sx1276.h"
#include "MPU9250.h"
#include "mpu9255.h"
#include "MS5611.h"
#include "w25qxx.h"
#include "File_Handling.h"
#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESERVE 0
#define PACKET_LEN 118
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId task1secHandle;
osThreadId myTask03Handle;
/* USER CODE BEGIN PV */
// Служебные переменные дл�? NMEA

int gga = 0;
int k = 0;
int zapyat = 0;
char NMEA[246] = {};
char GGA[80] = {};
char str[512] = {};
char str1024[1536] = {};
uint8_t bufferia[512] = {};
uint8_t packetint[4][39] = {{'9','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1'},
                            {'9','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2'},
                            {'9','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3'},
                            {'9','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4'}};
uint8_t picoN = 0;
uint8_t negrevatel = 0;
uint8_t negrevatelirq = 0;
uint32_t currtimenegrev = 0;
MPU9255_t MPU9255;
lora_sx1276 lora;
////////////////////////////////////////////////////////
uint8_t counterPressure = 0; // 5 sec po 5 izmereniy = 25 counter and alt > 3km
uint8_t commandbyte = 7; // to 0 to start kotovasiya
uint8_t stepcounter = 0; // up to 200, brat po %2, kogda == 0 to step delat (vozmojnost' rashireniya do bolee malenkih shagov)
uint16_t rotatorspeed = 940;
uint8_t w25tosd = 0;
////////////////////////////////////////////////////////
float temp1, temp2, temp3, temp4, lat1, lat2, lat3, lat4, lon1, lon2, lon3, lon4, alt1, alt2, alt3, alt4;
float altitude = 0;
float altitude0 = 0;
// 202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M
// 		   9,		 9,1,        10,1,1, 2,  3,      7,1

struct GPGGA_Struct         // Структура �? данными GPS-ГЛО�?�?СС-модул�?
{
  	  char Time [sizeof("hhmmss.ss")+RESERVE];      // Врем�?
  	  char Latitude [sizeof("xxxx.yyyyyy")+RESERVE];    // Широта
  	  char NS[sizeof("N")];                            // Север-Юг
  	  char Longitude[sizeof("xxxxx.yyyyyy")+RESERVE]; // Долгота
  	  char WE[sizeof("W")+RESERVE];                            // Запад-Во�?ток
  	  char Qual[sizeof("1")+RESERVE];               // Режим работы приемника
  	  char Sats[sizeof("nn")+RESERVE];     // Количе�?тво �?путников в решении
  	  char Alt[sizeof("aaaaa.a")+RESERVE];    // Antenna altitude above/below mean sea leve
  	  char Hdop[sizeof("xxx")+RESERVE];          // Horizontal dilution of precision
  	  char Units[sizeof("m")+RESERVE];         // Units of antenna altitude (M = metres)
};
struct GPGGA_Struct GPGGA = {
		"hhmmss.ss",
		"1111.111111",
		"N",
		"11111.111111",
		"W",
		"1",
		"nn",
		"aaaaa.a",
		"xxx",
		"m"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET	);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  //task1secHandle = osThreadNew(StartDefaultTask, NULL, &task1sec_attributes);

  //myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of task1sec */
  osThreadDef(task1sec, StartDefaultTask, osPriorityAboveNormal, 0, 1024);
  task1secHandle = osThreadCreate(osThread(task1sec), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityRealtime, 0, 1024);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




    /*

	*/
      //MS5611_t datastruct;
      //MS5611_init(&hi2c1, &datastruct);


  while (1)
  {

	  //float t = Ds18b20_ManualConvert();
	  //MS5611_read_press(&hi2c1, &datastruct, 0x48);
	  //MS5611_calculate(&datastruct);


	  //printf("W25 Id %d\n", W25qxx_ReadID());
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1440-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 72-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DS18B20_Pin|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CS_Pin */
  GPIO_InitStruct.Pin = SDIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void floatToByte(uint8_t* bytes, float f){
  int length = sizeof(float);
  for(int i = 0; i < length; i++){
    bytes[i] = ((uint8_t*)&f)[i];
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1){
		CDC_Transmit_FS(NMEA, 246);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
		printf("Obrabotaal\n");
		NMEA_Handler();
		HAL_UART_Receive_DMA(&huart1, NMEA, 246);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//printf("\nIRQ %ld\n", HAL_GetTick());
	//lora_mode_standby(&lora);
    if(GPIO_Pin == GPIO_PIN_4) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	//lora_mode_receive_single(&lora); // Toggle The Output (LED) Pin
    	uint8_t res = 0;
    	uint8_t buffercurren[117];
    			    // Put LoRa modem into continuous receive mode

    			    // Wait for packet up to 10sec

    	uint8_t len = lora_receive_packet(&lora, &buffercurren, sizeof(buffercurren), &res);
    	lora_clear_interrupt_rx_all(&lora);
    	memset(str, 0, sizeof(str));
    	sprintf(str, "\nInto checking...\n");
    	CDC_Transmit_FS(str, 512);
		if (res != LORA_OK) {
	    	memset(str, 0, sizeof(str));
	    	sprintf(str, "\NOT GUD ???...\n");
	    	CDC_Transmit_FS(str, 512);
	    } else {
	    	if (len%39 == 0){
	    		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
	    		memset(str, 0, sizeof(str));
	    		sprintf(str, "\nInto checking...\n");
	    		CDC_Transmit_FS(str, 512);
	    		for (int j = 0; j < (uint8_t)(len/39); j++){
	    			memset(str, 0, sizeof(str));
	    			sprintf(str, "\nInto checking...\n");
	    			CDC_Transmit_FS(str, 512);
	    			if (buffercurren[j*39] == '1'){
	    				for(int i = 0; i < 39; i++){
	    					packetint[0][i] = (uint8_t)(buffercurren[(j*39)+i]);
	    				}
	    				memset(str, 0, sizeof(str));
	    				sprintf(str, "\nGot first\n");
	    				CDC_Transmit_FS(str, 512);
	    				CDC_Transmit_FS(packetint[0], 118);
	    				if (picoN < 4) picoN += 1;
	    			}
	    			if (buffercurren[j*39] == '2'){
	    				for(int i = 0; i < 39; i++){
	    					packetint[1][i] = (uint8_t)(buffercurren[(j*39)+i]);
	    				}
	    				memset(str, 0, sizeof(str));
	    				sprintf(str, "*\nGot second");
	    				CDC_Transmit_FS(str, 512);
	    				CDC_Transmit_FS(packetint[1], 118);
	    				if (picoN < 4) picoN += 1;
	    			}
	    			if (buffercurren[j*39] == '3'){
	    				for(int i = 0; i < 39; i++){
	    					packetint[2][i] = (uint8_t)(buffercurren[(j*39)+i]);
	    				}
	    				memset(str, 0, sizeof(str));
	    				sprintf(str, "*\nGot third");
	    				CDC_Transmit_FS(str, 512);
	    				CDC_Transmit_FS(packetint[2], 118);
	    				if (picoN < 4) picoN += 1;
	    			}
	    			if (buffercurren[j*39] == '4'){
	    				for(int i = 0; i < 39; i++){
	    					packetint[3][i] = (uint8_t)(buffercurren[(j*39)+i]);
	    				}
	    				memset(str, 0, sizeof(str));
	    				sprintf(str, "*\nGot fourth");
	    				CDC_Transmit_FS(str, 512);
	    				CDC_Transmit_FS(packetint[3], 118);
	    				if (picoN < 4) picoN += 1;
	    			}
	    		}
		    } else if (len == 4){
		    	if (buffercurren[0] == 'a' && buffercurren[1] == 'b'){
			    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
			    	commandbyte = 0;
		    	}
		    	if (buffercurren[0] == 's' && buffercurren[1] == 'e'){
			    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
			    	commandbyte = 7;
		    	}
		    	if (buffercurren[0] == 'c' && buffercurren[1] == 'd'){
			    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
			    	if (rotatorspeed == 940) rotatorspeed = 910;
			    	else if (rotatorspeed == 910) rotatorspeed = 940;
		    	}
		    	if (buffercurren[0] == 'r' && buffercurren[1] == 's'){
		    		w25tosd = 1;
		    	}
		    	if (buffercurren[0] == 't' && buffercurren[1] == 'n'){
		    		/*
		    		if (negrevatel == 1){
		    			if(HAL_GetTick() - currtimenegrev > 19999){

		    			}
		    		}
		    		if (negrevatel == 0){
		    			currtimenegrev = HAL_GetTick();
		    			negrevatel = 1;
		    		}
		    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		    		*/
		    		negrevatelirq = 1;
				}
		    	if (buffercurren[0] == 't' && buffercurren[1] == 'o'){
		    		negrevatelirq = 2;
		    		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				}
		    }
	    }
    	memset(str, 0, sizeof(str));
    	//sprintf(str, "*\nTime: %d, len: %d, %c, %d", HAL_GetTick(), len, buffercurren[0], picoN);
    	//CDC_Transmit_FS(str, 512);

    	/*
    	uint8_t bufferewr[144];
    			   // Put LoRa modem into continuous receive mode
    			   // Wait for packet up to 10sec
    	uint8_t res;
    			 uint8_t len = lora_receive_packet(&lora, bufferewr, sizeof(bufferewr), &res);
    			 if (res != LORA_OK) {
    				 printf("Not caught\n");
    			 } else {
    				 printf("Caught\n");
    			 }
		*/
    }
}
int _write(int file, char *ptr, int len) {
    static uint8_t rc = USBD_OK;
    /*
    do {
        rc = CDC_Transmit_FS((uint8_t*)ptr, len);
    } while (USBD_BUSY == rc);
    */
    rc = CDC_Transmit_FS((uint8_t*)ptr, len);
    if (USBD_FAIL == rc) {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
}
int32_t UTCToInt(char* arr){
    uint8_t counteria = 0;
    char sad[9] = "00000000";
    for(uint8_t i = 0; i < 9; i++){
        if (arr[i] != '.'){
            sad[counteria] = arr[i];
            counteria += 1;
        } else {
            continue;
        }
    }
    return atol(sad);
}
/*
int32_t GPSToIntDepreciated(char* arr, uint8_t mode){
    if (mode==0){
        char sad[11] = "0000000000";
        uint8_t counteria = 0;
        for(uint8_t i = 0; i < 11; i++){
            if (arr[i] != '.'){
                sad[counteria] = arr[i];
                counteria += 1;
            } else {
                continue;
            }
        }
        return atol(sad);
    }
    if (mode==1){
        char sad[12] = "00000000000";
        uint8_t counteria = 0;
        for(uint8_t i = 0; i < 12; i++){
            if (arr[i] != '.'){
                sad[counteria] = arr[i];
                counteria += 1;
            } else {
                continue;
            }
        }
        return atol(sad);
    }
}
*/
void GPSToInt(const char* arr, char* arrhelp, uint8_t mode){
    if(mode == 0){

        for(uint8_t i = 0; i < 10; i++){
            arrhelp[i] = arr[i];
            printf("%c", arrhelp[i]);
        }

        memmove(arrhelp+4, arrhelp+4+1, strlen(arrhelp) - 4);
    }
    if(mode == 1){
        for(uint8_t i = 0; i < 11; i++){
            arrhelp[i] = arr[i];
            printf("%c", arrhelp[i]);
        }
        printf("Next\n");
        memmove(arrhelp+5, arrhelp+5+1, strlen(arrhelp) - 5);
        for(uint8_t i = 0; i < 11; i++){
            printf("%c", arrhelp[i]);
        }

    }
}

float PressToHeight(int32_t p){
	return 44330 * (1.0 - pow((float)p/101325, 0.1903));
}
int counter1 = 0;
int kolcbuff = 0;
int iteratorstate = 0;
uint8_t flashBlock[236] = {};
uint32_t counterPage = 0;
uint8_t counterPackInPage = 0;
int16_t AccData[3] = {};
int16_t GyroData[3] = {};
int16_t MagData[3] = {};
int32_t b;
uint8_t acknowledge = 0;

uint8_t tempp[4];
uint8_t tempp1[4];
uint8_t tempp2[4];
uint8_t tempp3[4];
uint8_t tempp4[4];
unsigned int longitude;
unsigned int latitude;
uint8_t appendbyte;
uint8_t packet[118] = {1};
uint8_t firstsend = 1;
uint8_t SDUnpluged = 0;
uint32_t valueADC[5] = {0};
char lat111[12] = {};
char lon111[12] = {};
char GGAhelp[80] = {};
uint8_t iter1 = 0;
void NMEA_Handler(){
	if (iter1){
	    // Обработка о�?тавшего�?�? пакета
	    strncpy(GGA, GGAhelp, iter1);
	    for (int b = 0; b < 55-iter1+2; b++){
	        GGA[iter1+b-1] = NMEA[b];
	    }
	    gga = 1;
	    iter1 = 0;
	}
	if (!gga)
	for (int i = 0; i < 246; i++){
		if (counter1 == 6)
		{ // $xxGGA пойман
				if ((245 - i) > 55){
				    counter1 = 0;
					for (int b = 0; b < 55; b++){
						GGA[b] = NMEA[b+i+1];
					}
					gga = 1;
					iter1 = 0;
					break;
				} else {
					iter1 = 245 - i + 1;
					iteratorstate = 55-(245-iter1);
					for (int b = 0; b < 246-i; b++){
						GGA[b] = NMEA[i+b+1];
					}
					strncpy(GGAhelp, GGA, iter1);
					counter1 = 0;
                    break;
				}
		}
		if (counter1 == 0){
		    if (NMEA[i] == '$'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != '$'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 1) {counter1++; continue;}
		if (counter1 == 2) {counter1++; continue;}
		if (counter1 == 3){
		    if (NMEA[i] == 'G'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != 'G'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 4){
		    if (NMEA[i] == 'G'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != 'G'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 5){
		    if (NMEA[i] == 'A'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != 'A'){
		        counter1 = 0;
		        continue;
		    }
		}
	}
	if (gga){
		// 9 ","
		uint8_t zapyat = 0;
		uint8_t k = 0;
		for (int i = 0; i < 56; i++){
			if (zapyat > 8){
				break;
			}
			if (GGA[i] == ','){
				zapyat += 1;
				k = 0;
				continue;
			}
			else{
				if (zapyat == 0 && k < sizeof("hhmmss.ss")+RESERVE-1){ GPGGA.Time[k] = GGA[i]; k++;}
				if (zapyat == 1 && k < sizeof("xxxx.yyyyyy")+RESERVE-1){ GPGGA.Latitude[k] = GGA[i]; k++;}
				if (zapyat == 2 && k < sizeof("N")+RESERVE-1){ GPGGA.NS[k] = GGA[i]; k++;}
				if (zapyat == 3 && k < sizeof("xxxxx.yyyyyy")+RESERVE-1){ GPGGA.Longitude[k] = GGA[i]; k++;}
				if (zapyat == 4 && k < sizeof("W")+RESERVE-1){ GPGGA.WE[k] = GGA[i]; k++;}
				if (zapyat == 5 && k < sizeof("1")+RESERVE-1){ GPGGA.Qual[k] = GGA[i]; k++;}
				if (zapyat == 6 && k < sizeof("nn")+RESERVE-1){ GPGGA.Sats[k] = GGA[i]; k++;}
				if (zapyat == 7 && k < sizeof("xxx")+RESERVE-1){ GPGGA.Hdop[k] = GGA[i]; k++;}
				if (zapyat == 8 && k < sizeof("aaaa.aa")+RESERVE-1){ GPGGA.Alt[k] = GGA[i]; k++;}
				if (zapyat == 9 && k < sizeof("m")+RESERVE-1){ GPGGA.Units[k] = GGA[i]; k++;}
			}
		}
		gga = 0;
		k = 0;
		zapyat = 0;
	}

}
void NMEA_HandlerLegacy(){
	//sizeof NMEA = 246, sizeof GGA = 53 => if i > 245-53 corrupt found, �?ледовательно
	//чтобы получить ча�?ть обрыва требует�?�? найти разно�?ть и-той, о�?тавшую�?�? ча�?ть отдельно обработать
	//пример: i = 240, 245 - 240 = 5, в текущем пакете обработать 5 (до NMEA[i+5]), �?ледущий прочитать 53-5+1 = 49,
	//n = 245 - i%245 and i%245
	//Может �?чётчик какой тип пакета �?ейча�?? От�?читывать �?чётчик по�?ледовательно�?ти, е�?ли разрыв - �?тереть,
	//Тое�?ть $xx был в первом пакете, �?то было отмечено как 3, так как по�?ледовательно верные чи�?ла, �?ледовательно
	//�?читать �? тройки, а значить далее провер�?ть GGA
	//Как только значение полна�? ше�?тёрка, тогда �?читывать по�?ледовательно в�?е �?ледующие �?имволы �? таким же �?четчиком,
	//От�?леживать размер текущего �?четчика и мак�?имального, предполагаю через %
	//Итого - в ма�?�?иве NMEA должен быть круговой ма�?�?ив, у которого будет от�?леживать�?�? �?о�?то�?ние круга
	for (int i = 0; i < 246; i++){
	    printf("\ncounter %d\n", counter1);
		if (counter1 == 6)
		{ // $xxGGA пойман
				printf("Gotcha %d\n", i);
				counter1 = 0;
				if ((246 - i) > 53){
					for (int b = 0; b < 53; b++){
						GGA[b] = NMEA[b+i+1];
					}
					gga = 1;
					printf("\n%s", GGA);
					break;
				} else { //snachala prochitat' tekushie znacheniya, potom uzhe na sleduyushei iteracii ostavshiesya: 53 - (246-i)
					iteratorstate = 246-i;
					if (kolcbuff < 2){
						if (kolcbuff == 0){
							for (int b = i; b < 246; b++){
								GGA[b-i] = NMEA[b+1];
							}
						} else {
							for (int b = 0; b < 53 - (246-i); b++){
								GGA[b+(246-i)] = NMEA[(b)%246];
							}
						}
						kolcbuff += 1;
					}
					if (kolcbuff >= 2){
						kolcbuff = 0;
						gga = 1;
					}
					break;
				}
		}
		if (counter1 == 0){
		    if (NMEA[i] == '$'){
		        printf(" %d", i);
			    counter1++;
			    printf("%c", NMEA[i]);
			    printf("%c", NMEA[i+1]);
			    printf("%c", NMEA[i+2]);
			    printf("%c", NMEA[i+3]);
			    printf("%c", NMEA[i+4]);
			    printf("%c", NMEA[i+5]);
			    continue;
		    } else if (NMEA[i] != 'A'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 1) {counter1++; printf("%d", counter1); printf("%c", NMEA[i]); continue;}
		if (counter1 == 2) {counter1++; printf("%d", counter1); printf("%c", NMEA[i]); printf("%c", NMEA[i+1]); continue;}
		if (counter1 == 3){
		    if (NMEA[i] == 'G'){
		        printf(" %d", i);
			    counter1++;
			    printf("%d", counter1);
			    continue;
		    } else if (NMEA[i] != 'G'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 4){
		    if (NMEA[i] == 'G'){
		        printf(" %d", i);
			    counter1++;
			    printf("%d", counter1);
			    continue;
		    } else if (NMEA[i] != 'G'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 5){
		    if (NMEA[i] == 'A'){
		        printf(" %d", i);
			    counter1++;
			    printf("%d", counter1);
			    continue;
		    } else if (NMEA[i] != 'A'){
		        counter1 = 0;
		        continue;
		    }
		}
	}
	if (gga){
		// 9 ","
		uint8_t zapyat = 0;
		uint8_t k = 0;
		for (int i = 0; i < 53; i++){
			if (zapyat > 9){
				break;
			}
			if (GGA[i] == ','){
				zapyat += 1;
				k = 0;
				continue;
			}
			else{
				if (zapyat == 0 && k < sizeof("hhmmss.ss")+RESERVE-1){ GPGGA.Time[k] = GGA[i]; k++;}
				if (zapyat == 1 && k < sizeof("xxxx.yyyyyy")+RESERVE-1){ GPGGA.Latitude[k] = GGA[i]; k++;}
				if (zapyat == 2 && k < sizeof("N")-1){ GPGGA.NS[k] = GGA[i]; k++;}
				if (zapyat == 3 && k < sizeof("xxxxx.yyyyyy")+RESERVE-1){ GPGGA.Longitude[k] = GGA[i]; k++;}
				if (zapyat == 4 && k < sizeof("W")-1){ GPGGA.WE[k] = GGA[i]; k++;}
				if (zapyat == 5 && k < sizeof("1")-1){ GPGGA.Qual[k] = GGA[i]; k++;}
				if (zapyat == 6 && k < sizeof("nn")+RESERVE-1){ GPGGA.Sats[k] = GGA[i]; k++;}
				if (zapyat == 7 && k < sizeof("xxx")+RESERVE-1){ GPGGA.Hdop[k] = GGA[i]; k++;}
				if (zapyat == 8 && k < sizeof("aaaa.aa")+RESERVE-1){ GPGGA.Alt[k] = GGA[i]; k++;}
				if (zapyat == 9 && k < sizeof("m")+RESERVE-1){ GPGGA.Units[k] = GGA[i]; k++;}
			}
		}
		gga = 0;
		k = 0;
		zapyat = 0;
	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the task1sec thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  BSP_SD_Init();
  HAL_UART_Receive_DMA(&huart1, NMEA, 246);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  TIM5->CCR4 = 925;
  /*
   * PWM = 50Hz / 20ms
   * 1000% = 20ms
   * 50% = 1ms == CCRx = 50
   * formula: (ms*1000/20), where ms = number of mseconds
   *
   * в по�?то�?нном вращении 0 - одна �?торона, 180 - друга�?, 90 - �?топ.
   * е�?ли поворот на 360 - 1.14�?, то поворот на 90 - 0.285�?, поворот на 45 - 0.1425�?
   * Дл�? инверт:
   * 975 (max-25) - 0.5ms = 0
   * 925 (max-75) - 1.5ms = 90
   * 875 (max-125) - 2.5ms = 180
   *
   * на поворот 360 граду�?ов - 2 �?еки
   * на 60 граду�?ов - 1/3
   * на 90 граду�?ов - 0.5
   *
   * 25 - 0.5ms = 0
   * 75 - 1.5ms = 90
   * 125 - 2.5ms = 180
   * 75 - 1.5ms = 90
   *
   *
   * */
  osDelay(1000);





  //TIM5->CCR4 = 940;
  //osDelay(550);
  //TIM5->CCR3 = 925;
  TIM5->CCR4 = 925;
  osDelay(1000);


  //TIM5->CCR3 = 925;
  //TIM5->CCR4 = 925;
  reset();
  //begin(915E6, 17);
  W25qxx_Init();
  uint8_t res = lora_init(&lora, &hspi1, GPIOA, GPIO_PIN_14, 436E6);
  if (res != LORA_OK) {
	  printf("Lora ne podkluchilas T_T\n");
  } else {
	printf("Lore horosho)\n");
  }
  lora_enable_interrupt_rx_done(&lora);

  HAL_SD_CardInfoTypeDef Card_Info;
  int8_t const card_type[3][15] = {"CARD_SDSC\0","CARD_SDHC_SDXC\0","CARD_SECURED\0"};
  int8_t const card_ver [2][10] = {"CARD_V1_X\0","CARD_V1_X\0"};
  volatile FRESULT fres;


  ADXL_ConfigTypeDef_t ADXL;
  ADXL345_StandbyON();
  ADXL.BWRate = BWRATE_1600;
  ADXL.PowerMode = NormalPower;
  ADXL.Format.Range = RANGE_4G;
  ADXL345_Init(&ADXL);
  ADXL345_MeasureON();
  Ds18b20_Init(osPriorityHigh);
  printf("\n\rStart testing SDCARD\n\r");
  osDelay(5000);
  BSP_SD_GetCardInfo(&Card_Info);
  memset(str, 0, sizeof(str));
  sprintf(str, "Card Type               -> %s\n", card_type[Card_Info.CardType]);
  CDC_Transmit_FS(str, 128);
  memset(str, 0, sizeof(str));
  sprintf(str, "Card Version            -> %s\n", card_ver[Card_Info.CardVersion]);
  CDC_Transmit_FS(str, 128);
  memset(str, 0, sizeof(str));
  sprintf(str, "Block Size              -> 0x%x\n", (int)Card_Info.BlockSize);
  CDC_Transmit_FS(str, 128);
  memset(str, 0, sizeof(str));
  sprintf(str, "Card Capacity in blocks -> 0x%x(%uGB)\n\r", (int)Card_Info.BlockNbr,(int)((((float)Card_Info.BlockNbr/1000)*(float)Card_Info.BlockSize/1000000)+0.5));
  CDC_Transmit_FS(str, 128);
  memset(str, 0, sizeof(str));
  for (int i = 0; i < 10; i++){
	  L3G4200D_init();
	  osDelay(1000);
  }
  MMC5883_init();



  //MPU9250_Init();
  //MPU9255_Init(&hi2c1);
  HAL_ADC_Start_DMA(&hadc1, valueADC, 5);
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();
  //osDelay(3000);

  /* Infinite loop */
  for(;;){
	  if (w25tosd){
		  w25tosd = 0;
		  //vTaskSuspendAll();
			//Create_File("W25DATA.TXT");
			Mount_SD("/");
			uint8_t datatotrans[236] = {0};
			for(int i = 0; i < 256; i++){
				 W25qxx_ReadPage(datatotrans, i, 0, 236);
				for(int j = 0; j < 2; j++){
					 int16_t Axj = ((uint8_t)datatotrans[j*118+0+1] << 8 | (uint8_t)datatotrans[j*118+1+1]);
					 int16_t Ayj = ((uint8_t)datatotrans[j*118+0+3] << 8 | (uint8_t)datatotrans[j*118+1+3]);
					 int16_t Azj = ((uint8_t)datatotrans[j*118+0+5] << 8 | (uint8_t)datatotrans[j*118+1+5]);
					 int16_t Gxj = ((uint8_t)datatotrans[j*118+0+7] << 8 | (uint8_t)datatotrans[j*118+1+7]);
					 int16_t Gyj = ((uint8_t)datatotrans[j*118+0+9] << 8 | (uint8_t)datatotrans[j*118+1+9]);
					 int16_t Gzj = ((uint8_t)datatotrans[j*118+0+11] << 8 | (uint8_t)datatotrans[j*118+1+11]);
					 int16_t Mxj = ((uint8_t)datatotrans[j*118+0+13] << 8 | (uint8_t)datatotrans[j*118+1+13]);
					 int16_t Myj = ((uint8_t)datatotrans[j*118+0+15] << 8 | (uint8_t)datatotrans[j*118+1+15]);
					 int16_t Mzj = ((uint8_t)datatotrans[j*118+0+17] << 8 | (uint8_t)datatotrans[j*118+1+17]);

					 float tempj;
					 float temp1j;
					 float temp2j;
					 float temp3j;
					 float temp4j;
					 int32_t barj;
					 int32_t latj;
					 int32_t longij;
					 uint8_t temparrj[4] = {(uint8_t)datatotrans[j*118+19], (uint8_t)datatotrans[j*118+20], (uint8_t)datatotrans[j*118+21], (uint8_t)datatotrans[j*118+22]};
					 memcpy(&tempj, &temparrj, sizeof(tempj));
					 uint8_t temparr1j[4] = {(uint8_t)datatotrans[j*118+23], (uint8_t)datatotrans[j*118+24], (uint8_t)datatotrans[j*118+25], (uint8_t)datatotrans[j*118+26]};
					 memcpy(&temp1j, &temparr1j, sizeof(tempj));
					 uint8_t temparr2j[4] = {(uint8_t)datatotrans[j*118+27], (uint8_t)datatotrans[j*118+28], (uint8_t)datatotrans[j*118+29], (uint8_t)datatotrans[j*118+30]};
					 memcpy(&temp2j, &temparr2j, sizeof(tempj));
					 uint8_t temparr3j[4] = {(uint8_t)datatotrans[j*118+31], (uint8_t)datatotrans[j*118+32], (uint8_t)datatotrans[j*118+33], (uint8_t)datatotrans[j*118+34]};
					 memcpy(&temp3j, &temparr3j, sizeof(tempj));
					 uint8_t temparr4j[4] = {(uint8_t)datatotrans[j*118+35], (uint8_t)datatotrans[j*118+36], (uint8_t)datatotrans[j*118+37], (uint8_t)datatotrans[j*118+38]};
					 memcpy(&temp4j, &temparr4j, sizeof(tempj));
					 uint8_t bararrj[4] = {(uint8_t)datatotrans[j*118+42], (uint8_t)datatotrans[j*118+41], (uint8_t)datatotrans[j*118+40], (uint8_t)datatotrans[j*118+39]};
					 memcpy(&barj, &bararrj, sizeof(barj));
					 uint8_t latarrj[4] = {(uint8_t)datatotrans[j*118+46], (uint8_t)datatotrans[j*118+45], (uint8_t)datatotrans[j*118+44], (uint8_t)datatotrans[j*118+43]};
					 uint8_t longarrj[4] = {(uint8_t)datatotrans[j*118+50], (uint8_t)datatotrans[j*118+49], (uint8_t)datatotrans[j*118+48], (uint8_t)datatotrans[j*118+47]};
					 memcpy(&latj, &latarrj, sizeof(latj));
					 memcpy(&longij, &longarrj, sizeof(longij));
					 memset(str, 0, sizeof(str));
					 sprintf(str, "DS0: %f DS1: %f DS2: %f DS3: %f DS4: %f Press: %d\n Accx: %f Accy: %f Accz: %f\n Magx: %f Magy: %f Magz: %f\n Gyrox: %f Gyroy %f Gyroz %f\n",
												  tempj,
												  temp1j,
												  temp2j,
												  temp3j,
												  temp3j, barj,
												  Axj*1/256.0f, Ayj*1/256.0f, Azj*1/256.0f,
												  Mxj*0.025f, Myj*0.025f, Mzj*0.025f,
												  Gxj*.00875f, Gyj*.00875f, Gzj*.00875f
					  );
					 Update_File("W25DATA.TXT", str);
					 Update_File("W25DATA.TXT", datatotrans);
	    			}
	    		}
	    		Unmount_SD("/");
	    		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
	    		//xTaskResumeAll();

	  }
	  b = meas();
	  if (negrevatelirq == 1){
  		if (negrevatel == 1){
  			if(HAL_GetTick() - currtimenegrev > 19999){
  				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  				negrevatel = 0;
  				negrevatelirq = 0;
  			}
  		}
  		if (negrevatel == 0 && negrevatelirq == 1){
  			currtimenegrev = HAL_GetTick();
  	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  			negrevatel = 1;
  		}
	  }
	  if (negrevatelirq == 2){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		negrevatelirq = 0;
	  }
	  if (altitude0 == 0) altitude0 = PressToHeight(b);
	  if ((altitude - PressToHeight(b) > 0) && (altitude - altitude0 > 10000)){
		  counterPressure += 1;
		  if (counterPressure == 25) {
			  commandbyte = 0;
			  counterPressure = 0;
		  }
	  } else {
		  counterPressure = 0;
	  }
	  altitude = PressToHeight(b);
	 //MPU9250_GetData(AccData, MagData, GyroData);
	 //readAll(&hi2c1, &MPU9255);
	 AccData[0] = ADXL345_GetValue(Xaxis);
	 AccData[1] = ADXL345_GetValue(Yaxis);
	 AccData[2] = ADXL345_GetValue(Zaxis);
	 MMCgetMag(MagData);
	 /*
	 MagData[0] = MMCgetMAGX(); //* (float)MMC5883MA_DYNAMIC_RANGE / MMC5883MA_RESOLUTION - (float)MMC5883MA_DYNAMIC_RANGE / 2;
	 MagData[1] = MMCgetMAGY(); //* (float)MMC5883MA_DYNAMIC_RANGE / MMC5883MA_RESOLUTION - (float)MMC5883MA_DYNAMIC_RANGE / 2;
	 MagData[2] = MMCgetMAGZ(); //* (float)MMC5883MA_DYNAMIC_RANGE / MMC5883MA_RESOLUTION - (float)MMC5883MA_DYNAMIC_RANGE / 2;
	 */
	 L3GgetGyro();
	 GyroData[0] = L3GgetGyroX();
	 GyroData[1] = L3GgetGyroY();
	 GyroData[2] = L3GgetGyroZ();

	 //uint32_t valueADC[4];
	 //HAL_ADC_Start(&hadc1, valueADC, 4);
	 //HAL_ADC_PollForConversion(&hadc1, 1000);
	 //raw = HAL_ADC_GetValue(&hadc1);


	 		 floatToByte(tempp, ds18b20[0].Temperature);
	 		 floatToByte(tempp1, ds18b20[1].Temperature);
	 		 floatToByte(tempp2, ds18b20[2].Temperature);
	 		 floatToByte(tempp3, ds18b20[3].Temperature);
	 		 floatToByte(tempp4, ds18b20[4].Temperature);
	 		 GPSToInt(GPGGA.Longitude, lon111, 1);
	 		 GPSToInt(GPGGA.Latitude, lat111, 0);
	 		 longitude = atoi(lon111);
	 		 latitude = atoi(lat111);
	 		 appendbyte = GPGGA.Longitude[11];
	 		 uint8_t packettocopy[54+16*picoN];
	 		 uint8_t packet1[54] = {commandbyte, (AccData[0] >> 8) & 0xFF, AccData[0], (AccData[1] >> 8) & 0xFF, AccData[1], (AccData[2] >> 8) & 0xFF, AccData[2],
	 				 	 	 	   (GyroData[0] >> 8) & 0xFF, GyroData[0], (GyroData[1] >> 8) & 0xFF, GyroData[1], (GyroData[2] >> 8) & 0xFF, GyroData[2],
	 							   (MagData[0] >> 8) & 0xFF, MagData[0], (MagData[1] >> 8) & 0xFF, MagData[1], (MagData[2] >> 8) & 0xFF, MagData[2],
	 		 	 	 	 	 	   tempp[0], tempp[1],
	 							   tempp[2], tempp[3],
	 							   tempp1[0], tempp1[1],
	 							   tempp1[2], tempp1[3],
	 							   tempp2[0], tempp2[1],
	 							   tempp2[2], tempp2[3],
	 							   tempp3[0], tempp3[1],
	 							   tempp3[2], tempp3[3],
	 							   tempp4[0], tempp4[1],
	 							   tempp4[2], tempp4[3],
	 							   (uint8_t)((b >> 24) & 0xFF),
	 							   (uint8_t)((b >> 16) & 0xFF),
	 							   (uint8_t)((b >> 8) & 0xFF),
	 							   (uint8_t)((b >> 0) & 0xFF),
	 							   (uint8_t)((latitude >> 24) & 0xFF),
	 							   (uint8_t)((latitude >> 16) & 0xFF),
	 							   (uint8_t)((latitude >> 8) & 0xFF),
	 							   (uint8_t)((latitude >> 0) & 0xFF),
	 							   (uint8_t)((longitude >> 24) & 0xFF),
	 							   (uint8_t)((longitude >> 16) & 0xFF),
	 							   (uint8_t)((longitude >> 8) & 0xFF),
	 							   (uint8_t)((longitude >> 0) & 0xFF),
	 							   appendbyte,
	 							   (uint8_t)GPGGA.NS[0],
	 							   (uint8_t)GPGGA.WE[0]};
	 		 	 	 	 	 	 /*
	 							   GPGGA.Longitude[0], GPGGA.Longitude[1], GPGGA.Longitude[2], GPGGA.Longitude[3],
	 							   GPGGA.Longitude[4], GPGGA.Longitude[5], GPGGA.Longitude[6], GPGGA.Longitude[7],
	 							   GPGGA.Longitude[8], GPGGA.Longitude[9],
	 							   GPGGA.Latitude[0], GPGGA.Latitude[1], GPGGA.Latitude[2], GPGGA.Latitude[3],
	 							   GPGGA.Latitude[4], GPGGA.Latitude[5], GPGGA.Latitude[6], GPGGA.Latitude[7],
	 							   GPGGA.Latitude[8]
	 		 	 	 	 	 	   };
	 		 	 	 	 	 	 */
			memcpy(packettocopy, packet1, sizeof(packet1));
			for (int j = 0; j < picoN; j++){
				for (int p = 0; p < 16; p++){
					packettocopy[54+(j*16)+p] = packetint[j][(p+1)];
				}
			}
			//CDC_Transmit_FS(packettocopy, 118);
	 		memcpy(packet,packettocopy,54+16*picoN);
	 		memset(str, 0, sizeof(str));
	 		/*
			 sprintf(str, "Time %d DS0: %f DS1: %f DS2: %f DS3: %f DS4: %f Press: %d\n Accx: %f Accy: %f Accz: %f\n Magx: %f Magy: %f Magz: %f\n Gyrox: %f Gyroy %f Gyroz %f\n ADC 1: %d, ADC 2: %d, ADC 3: %d, ADC 4: %d, ADC 5: %d\n",
					 	 	 	 	 	  HAL_GetTick(),
					 	 	 	 	 	  ds18b20[0].Temperature,
			 		 	 				  ds18b20[1].Temperature,
			 		 	 				  ds18b20[2].Temperature,
			 		 	 				  ds18b20[3].Temperature,
			 		 	 				  ds18b20[4].Temperature, b,
			 		 	 				  //AccData[0]*(4.0 / 32768.0), AccData[1]*(4.0 / 32768.0), AccData[2]*(4.0 / 32768.0),
			 		 	 				  //GyroData[0]*1000.0 / 32768.0, GyroData[1]*1000.0 / 32768.0, GyroData[2]*1000.0 / 32768.0
			 		 	 				  AccData[0]*1/256.0f, AccData[1]*1/256.0f, AccData[2]*1/256.0f,
			 		 	 				  MagData[0]*0.025f, MagData[1]*0.025f, MagData[2]*0.025f,
			 		 	 				  GyroData[0] * .00875f, GyroData[1] * .00875f, GyroData[2] * .00875f,
										  valueADC[0], valueADC[1], valueADC[2], valueADC[3], valueADC[4]
			 		 	 		  );
			*/
			//CDC_Transmit_FS(str, 512);
			if (BSP_SD_IsDetected()){
				if (!Mount_SD("/") && SDUnpluged == 0){
					Update_File("FILE1.TXT", str);
					memset(str, 0, sizeof(str));
					memset(str1024, 0, sizeof(str));
					/*
					sprintf(str, "Pico 1 data: Number: %c, Pressure: %ld\nPico 2 data: Number: %c, Pressure: %ld\nPico 3 data: Number: %c, Pressure: %ld\nPico 4 data: Number: %c, Pressure: %ld\n",
							packetint[0][0],
							(int32_t)((packetint[0][35] << 8*3) | (packetint[0][36] << 8*2) | (packetint[0][37] << 8*1) | (packetint[0][38] << 8*0)),
							packetint[1][0],
							(int32_t)((packetint[1][35] << 8*3) | (packetint[1][36] << 8*2) | (packetint[1][37] << 8*1) | (packetint[1][38] << 8*0)),
							packetint[2][0],
							(int32_t)((packetint[2][35] << 8*3) | (packetint[2][36] << 8*2) | (packetint[2][37] << 8*1) | (packetint[2][38] << 8*0)),
							packetint[3][0],
							(int32_t)((packetint[3][35] << 8*3) | (packetint[3][36] << 8*2) | (packetint[3][37] << 8*1) | (packetint[3][38] << 8*0)));
					*/
					/*
					sprintf(str1024, "Pico 1: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\nPico 2: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\nPico 3: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\nPico 4: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\n",
							packetint[0][0],
							(float)((packetint[0][1] << 8*3) | (packetint[0][2] << 8*2) | (packetint[0][3] << 8*1) | (packetint[0][4] << 8*0)),
							(float)((packetint[0][5] << 8*3) | (packetint[0][6] << 8*2) | (packetint[0][7] << 8*1) | (packetint[0][8] << 8*0)),
							(float)((packetint[0][9] << 8*3) | (packetint[0][10] << 8*2) | (packetint[0][11] << 8*1) | (packetint[0][12] << 8*0)),
							(float)((packetint[0][13] << 8*3) | (packetint[0][14] << 8*2) | (packetint[0][15] << 8*1) | (packetint[0][16] << 8*0)),
							(int16_t)((packetint[0][17] << 8*1) | (packetint[0][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[0][19] << 8*1) | (packetint[0][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[0][21] << 8*1) | (packetint[0][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[0][23] << 8*1) | (packetint[0][24] << 8*0))*0.07,
							(int16_t)((packetint[0][25] << 8*1) | (packetint[0][26] << 8*0))*0.07,
							(int16_t)((packetint[0][27] << 8*1) | (packetint[0][28] << 8*0))*0.07,
							(int16_t)((packetint[0][29] << 8*1) | (packetint[0][30] << 8*0)),
							(int16_t)((packetint[0][31] << 8*1) | (packetint[0][32] << 8*0)),
							(int16_t)((packetint[0][33] << 8*1) | (packetint[0][34] << 8*0)),
							(int32_t)((packetint[0][35] << 8*3) | (packetint[0][36] << 8*2) | (packetint[0][37] << 8*1) | (packetint[0][38] << 8*0)),
							packetint[1][0],
							(float)((packetint[1][1] << 8*3) | (packetint[1][2] << 8*2) | (packetint[1][3] << 8*1) | (packetint[1][4] << 8*0)),
							(float)((packetint[1][5] << 8*3) | (packetint[1][6] << 8*2) | (packetint[1][7] << 8*1) | (packetint[1][8] << 8*0)),
							(float)((packetint[1][9] << 8*3) | (packetint[1][10] << 8*2) | (packetint[1][11] << 8*1) | (packetint[1][12] << 8*0)),
							(float)((packetint[1][13] << 8*3) | (packetint[1][14] << 8*2) | (packetint[1][15] << 8*1) | (packetint[1][16] << 8*0)),
							(int16_t)((packetint[1][17] << 8*1) | (packetint[1][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[1][19] << 8*1) | (packetint[1][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[1][21] << 8*1) | (packetint[1][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[1][23] << 8*1) | (packetint[1][24] << 8*0))*0.07,
							(int16_t)((packetint[1][25] << 8*1) | (packetint[1][26] << 8*0))*0.07,
							(int16_t)((packetint[1][27] << 8*1) | (packetint[1][28] << 8*0))*0.07,
							(int16_t)((packetint[1][29] << 8*1) | (packetint[1][30] << 8*0)),
							(int16_t)((packetint[1][31] << 8*1) | (packetint[1][32] << 8*0)),
							(int16_t)((packetint[1][33] << 8*1) | (packetint[1][34] << 8*0)),
							(int32_t)((packetint[1][35] << 8*3) | (packetint[1][36] << 8*2) | (packetint[1][37] << 8*1) | (packetint[1][38] << 8*0)),
							packetint[2][0],
							(float)((packetint[2][1] << 8*3) | (packetint[2][2] << 8*2) | (packetint[2][3] << 8*1) | (packetint[2][4] << 8*0)),
							(float)((packetint[2][5] << 8*3) | (packetint[2][6] << 8*2) | (packetint[2][7] << 8*1) | (packetint[2][8] << 8*0)),
							(float)((packetint[2][9] << 8*3) | (packetint[2][10] << 8*2) | (packetint[2][11] << 8*1) | (packetint[2][12] << 8*0)),
							(float)((packetint[2][13] << 8*3) | (packetint[2][14] << 8*2) | (packetint[2][15] << 8*1) | (packetint[2][16] << 8*0)),
							(int16_t)((packetint[2][17] << 8*1) | (packetint[2][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[2][19] << 8*1) | (packetint[2][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[2][21] << 8*1) | (packetint[2][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[2][23] << 8*1) | (packetint[2][24] << 8*0))*0.07,
							(int16_t)((packetint[2][25] << 8*1) | (packetint[2][26] << 8*0))*0.07,
							(int16_t)((packetint[2][27] << 8*1) | (packetint[2][28] << 8*0))*0.07,
							(int16_t)((packetint[2][29] << 8*1) | (packetint[2][30] << 8*0)),
							(int16_t)((packetint[2][31] << 8*1) | (packetint[2][32] << 8*0)),
							(int16_t)((packetint[2][33] << 8*1) | (packetint[2][34] << 8*0)),
							(int32_t)((packetint[2][35] << 8*3) | (packetint[2][36] << 8*2) | (packetint[2][37] << 8*1) | (packetint[2][38] << 8*0)),
							packetint[3][0],
							(float)((packetint[3][1] << 8*3) | (packetint[3][2] << 8*2) | (packetint[3][3] << 8*1) | (packetint[3][4] << 8*0)),
							(float)((packetint[3][5] << 8*3) | (packetint[3][6] << 8*2) | (packetint[3][7] << 8*1) | (packetint[3][8] << 8*0)),
							(float)((packetint[3][9] << 8*3) | (packetint[3][10] << 8*2) | (packetint[3][11] << 8*1) | (packetint[3][12] << 8*0)),
							(float)((packetint[3][13] << 8*3) | (packetint[3][14] << 8*2) | (packetint[3][15] << 8*1) | (packetint[3][16] << 8*0)),
							(int16_t)((packetint[3][17] << 8*1) | (packetint[3][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[3][19] << 8*1) | (packetint[3][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[3][21] << 8*1) | (packetint[3][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[3][23] << 8*1) | (packetint[3][24] << 8*0))*0.07,
							(int16_t)((packetint[3][25] << 8*1) | (packetint[3][26] << 8*0))*0.07,
							(int16_t)((packetint[3][27] << 8*1) | (packetint[3][28] << 8*0))*0.07,
							(int16_t)((packetint[3][29] << 8*1) | (packetint[3][30] << 8*0)),
							(int16_t)((packetint[3][31] << 8*1) | (packetint[3][32] << 8*0)),
							(int16_t)((packetint[3][33] << 8*1) | (packetint[3][34] << 8*0)),
							(int32_t)((packetint[3][35] << 8*3) | (packetint[3][36] << 8*2) | (packetint[3][37] << 8*1) | (packetint[3][38] << 8*0))
							);

					*/
					memcpy(&temp1, packetint[0]+1, sizeof(temp1));
					memcpy(&lat1, packetint[0]+5, sizeof(lat1));
					memcpy(&lon1, packetint[0]+9, sizeof(lon1));
					memcpy(&alt1, packetint[0]+13, sizeof(alt1));

					memcpy(&temp2, packetint[1]+1, sizeof(temp2));
					memcpy(&lat2, packetint[1]+5, sizeof(lat2));
					memcpy(&lon2, packetint[1]+9, sizeof(lon2));
					memcpy(&alt2, packetint[1]+13, sizeof(alt2));

					memcpy(&temp3, packetint[2]+1, sizeof(temp3));
					memcpy(&lat3, packetint[2]+5, sizeof(lat3));
					memcpy(&lon3, packetint[2]+9, sizeof(lon3));
					memcpy(&alt3, packetint[2]+13, sizeof(alt3));

					memcpy(&temp4, packetint[3]+1, sizeof(temp4));
					memcpy(&lat4, packetint[3]+5, sizeof(lat4));
					memcpy(&lon4, packetint[3]+9, sizeof(lon4));
					memcpy(&alt4, packetint[3]+13, sizeof(alt4));

					sprintf(str1024, "Alt: %f, %f Time %d DS0: %f DS1: %f DS2: %f DS3: %f DS4: %f Press: %d\n Accx: %f Accy: %f Accz: %f\n Magx: %f Magy: %f Magz: %f\n Gyrox: %f Gyroy %f Gyroz %f\n ADC 1: %d, ADC 2: %d, ADC 3: %d, ADC 4: %d, ADC 5: %d\nPico 1: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\nPico 2: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\nPico 3: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\nPico 4: %d, Temp: %f, Lat: %f, Lon: %f, Alt: %f, Ax, Ay, Az: %f, %f, %f, Gx, Gy, Gz: %f, %f, %f, Mx, My, Mz: %f, %f, %f, Press: %ld\n",
							altitude, altitude-altitude0,
							HAL_GetTick(),
							ds18b20[0].Temperature,
							ds18b20[1].Temperature,
							ds18b20[2].Temperature,
							ds18b20[3].Temperature,
							ds18b20[4].Temperature, b,
		 	 				  //AccData[0]*(4.0 / 32768.0), AccData[1]*(4.0 / 32768.0), AccData[2]*(4.0 / 32768.0),
		 	 				  //GyroData[0]*1000.0 / 32768.0, GyroData[1]*1000.0 / 32768.0, GyroData[2]*1000.0 / 32768.0
							AccData[0]*1/256.0f, AccData[1]*1/256.0f, AccData[2]*1/256.0f,
							MagData[0]*0.025f, MagData[1]*0.025f, MagData[2]*0.025f,
							GyroData[0] * .00875f, GyroData[1] * .00875f, GyroData[2] * .00875f,
							valueADC[0], valueADC[1], valueADC[2], valueADC[3], valueADC[4],
							packetint[0][0],
							temp1,
							lat1,
							lon1,
							alt1,
							(int16_t)((packetint[0][17] << 8*1) | (packetint[0][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[0][19] << 8*1) | (packetint[0][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[0][21] << 8*1) | (packetint[0][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[0][23] << 8*1) | (packetint[0][24] << 8*0))*0.07,
							(int16_t)((packetint[0][25] << 8*1) | (packetint[0][26] << 8*0))*0.07,
							(int16_t)((packetint[0][27] << 8*1) | (packetint[0][28] << 8*0))*0.07,
							(int16_t)((packetint[0][29] << 8*1) | (packetint[0][30] << 8*0))*1.f,
							(int16_t)((packetint[0][31] << 8*1) | (packetint[0][32] << 8*0))*1.f,
							(int16_t)((packetint[0][33] << 8*1) | (packetint[0][34] << 8*0))*1.f,
							(int32_t)((packetint[0][35] << 8*3) | (packetint[0][36] << 8*2) | (packetint[0][37] << 8*1) | (packetint[0][38] << 8*0)),
							packetint[1][0],
							temp2,
							lat2,
							lon2,
							alt2,
							(int16_t)((packetint[1][17] << 8*1) | (packetint[1][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[1][19] << 8*1) | (packetint[1][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[1][21] << 8*1) | (packetint[1][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[1][23] << 8*1) | (packetint[1][24] << 8*0))*0.07,
							(int16_t)((packetint[1][25] << 8*1) | (packetint[1][26] << 8*0))*0.07,
							(int16_t)((packetint[1][27] << 8*1) | (packetint[1][28] << 8*0))*0.07,
							(int16_t)((packetint[1][29] << 8*1) | (packetint[1][30] << 8*0))*1.f,
							(int16_t)((packetint[1][31] << 8*1) | (packetint[1][32] << 8*0))*1.f,
							(int16_t)((packetint[1][33] << 8*1) | (packetint[1][34] << 8*0))*1.f,
							(int32_t)((packetint[1][35] << 8*3) | (packetint[1][36] << 8*2) | (packetint[1][37] << 8*1) | (packetint[1][38] << 8*0)),
							packetint[2][0],
							temp3,
							lat3,
							lon3,
							alt3,
							(int16_t)((packetint[2][17] << 8*1) | (packetint[2][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[2][19] << 8*1) | (packetint[2][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[2][21] << 8*1) | (packetint[2][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[2][23] << 8*1) | (packetint[2][24] << 8*0))*0.07,
							(int16_t)((packetint[2][25] << 8*1) | (packetint[2][26] << 8*0))*0.07,
							(int16_t)((packetint[2][27] << 8*1) | (packetint[2][28] << 8*0))*0.07,
							(int16_t)((packetint[2][29] << 8*1) | (packetint[2][30] << 8*0))*1.f,
							(int16_t)((packetint[2][31] << 8*1) | (packetint[2][32] << 8*0))*1.f,
							(int16_t)((packetint[2][33] << 8*1) | (packetint[2][34] << 8*0))*1.f,
							(int32_t)((packetint[2][35] << 8*3) | (packetint[2][36] << 8*2) | (packetint[2][37] << 8*1) | (packetint[2][38] << 8*0)),
							packetint[3][0],
							temp4,
							lat4,
							lon4,
							alt4,
							(int16_t)((packetint[3][17] << 8*1) | (packetint[3][18] << 8*0))*1/256.0f,
							(int16_t)((packetint[3][19] << 8*1) | (packetint[3][20] << 8*0))*1/256.0f,
							(int16_t)((packetint[3][21] << 8*1) | (packetint[3][22] << 8*0))*1/256.0f,
							(int16_t)((packetint[3][23] << 8*1) | (packetint[3][24] << 8*0))*0.07,
							(int16_t)((packetint[3][25] << 8*1) | (packetint[3][26] << 8*0))*0.07,
							(int16_t)((packetint[3][27] << 8*1) | (packetint[3][28] << 8*0))*0.07,
							(int16_t)((packetint[3][29] << 8*1) | (packetint[3][30] << 8*0))*1.f,
							(int16_t)((packetint[3][31] << 8*1) | (packetint[3][32] << 8*0))*1.f,
							(int16_t)((packetint[3][33] << 8*1) | (packetint[3][34] << 8*0))*1.f,
							(int32_t)((packetint[3][35] << 8*3) | (packetint[3][36] << 8*2) | (packetint[3][37] << 8*1) | (packetint[3][38] << 8*0))
							);
					Update_File("FILE1.TXT", str1024);
					memset(str, 0, sizeof(str));
					sprintf(str, "%s;%s;%s;%s;%s;%s;%s\n", GPGGA.Time, GPGGA.Latitude, GPGGA.NS, GPGGA.Longitude, GPGGA.WE, GPGGA.Sats, GPGGA.Alt);
					Update_File("FILE2.TXT", str);
					memset(str, 0, sizeof(str));
					Unmount_SD("/");
				}
				if (SDUnpluged){
					printf("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
					MX_FATFS_Init();
					BSP_SD_Init();
					SDUnpluged = 0;
				}
			} else {
				if (SDUnpluged == 0){
					printf("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\n");
					SDUnpluged = 1;
					FATFS_UnLinkDriver(SDPath);
				}
			}
			memset(str, 0, sizeof(str));


			//CDC_Transmit_FS(str, 512);



	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	  osDelay(30000);
	  TickType_t xLastWakeTime;
	  const TickType_t xFrequency = 5000 / portTICK_PERIOD_MS;
	  xLastWakeTime = xTaskGetTickCount();
	  /* Infinite loop */
	  for(;;)
	  {

		  /* Accelerometer - 6 bytes
		   * Gyroscope - 6 bytes
		   * Magnitometer - 6 bytes
		   * Temperature - 20 bytes
		   * Barometer - 4 bytes
		   * Longitude - 5+1 bytes
		   * Latitude - 4+1 bytes
		   * ADC - 10 bytes
		   * PWM - 2 bytes
		   * Time - 4 bytes
		   * Alt - 7 bytes
		   * N of sats - 1 byte
		   * 77 bytes * 3 = 231 bytes per page.
		   */



		 uint8_t res;
		 res = lora_send_packet_blocking(&lora, packet, 118, 500);


		 if (res != LORA_OK) {
		 		 printf("Send err\n");
		 }
		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
		 lora_mode_receive_continuous(&lora);
		 memset(str, 0, sizeof(str));
		 if (!commandbyte && stepcounter < 400){
			 TIM5->CCR4 = rotatorspeed;
			 vTaskDelay(117);
			 TIM5->CCR4 = 925;
			 stepcounter += 1;
		 }
		 /*
		 sprintf(str, "Tim %d DS0: %f DS1: %f DS2: %f DS3: %f DS4: %f Press: %d\n Accx: %f Accy: %f Accz: %f\n Magx: %f Magy: %f Magz: %f\n Gyrox: %f Gyroy %f Gyroz %f\n",
				 	 	 	 	 	  HAL_GetTick(),
				 	 	 	 	 	  ds18b20[0].Temperature,
		 		 	 				  ds18b20[1].Temperature,
		 		 	 				  ds18b20[2].Temperature,
		 		 	 				  ds18b20[3].Temperature,
		 		 	 				  ds18b20[4].Temperature, b,
		 		 	 				  //AccData[0]*(4.0 / 32768.0), AccData[1]*(4.0 / 32768.0), AccData[2]*(4.0 / 32768.0),
		 		 	 				  //GyroData[0]*1000.0 / 32768.0, GyroData[1]*1000.0 / 32768.0, GyroData[2]*1000.0 / 32768.0
		 		 	 				  AccData[0]*1/256.0f, AccData[1]*1/256.0f, AccData[2]*1/256.0f,
		 		 	 				  MagData[0]*0.025f, MagData[1]*0.025f, MagData[2]*0.025f,
		 		 	 				  GyroData[0] * .00875f, GyroData[1] * .00875f, GyroData[2] * .00875f
		 		 	 		  );

		  //CDC_Transmit_FS(str, 256);
		  memset(str, 0, sizeof(str));
		  */
		  /*
		  uint8_t buffer[32];
		    // Put LoRa modem into continuous receive mode

		    // Wait for packet up to 10sec

		    uint8_t len = lora_receive_packet_blocking(&lora, buffer, sizeof(buffer), 10000, &res);
		    if (res != LORA_OK) {
		      // Receive failed
		    }
		    buffer[len] = 0;  // null terminate string to print it
		    printf("'%s'\n", buffer);
		   */

		 for (uint8_t i = 0; i < PACKET_LEN; i++){
			 flashBlock[i + PACKET_LEN*counterPackInPage] = packet[i];
		 }
		 counterPackInPage = (counterPackInPage+1)%2;
		 if (counterPage % 16 == 0) W25qxx_EraseSector((uint32_t)counterPage/16);
		 if (counterPackInPage == 0){
			 W25qxx_WritePage(flashBlock, counterPage, 0, PACKET_LEN*2);
			 memset(flashBlock, 0, sizeof(flashBlock));
			 W25qxx_ReadPage(flashBlock, counterPage, 0, PACKET_LEN*2);
			 CDC_Transmit_FS(flashBlock, PACKET_LEN*2);
			 counterPage += 1;
		 }


		  vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  }
  /* USER CODE END StartTask03 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
