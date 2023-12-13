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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "AMT22.h"
#include "RoboArm.h"
//#include "TMC2209.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId myAMT22TaskHandle;
osThreadId myUARTTaskHandle;
/* USER CODE BEGIN PV */

uint8_t rx_index = 0;
uint8_t rx_data[20];
uint8_t rx_buffer[12]; //!!!!!
uint8_t transfer_cplt = 0;

bool startFirstMove = false;
bool sendDataFlag = false;
bool stopBeforeReboot = false;
bool setZeroFlag = false;

bool timerFT1 = false;
bool timerFT2 = false;
bool correctPosFlag = false;

float recAngleF = 0.0;
uint16_t recDist = 0;
uint16_t recHold = 0;
bool flagReadEnc = 0;

int encMax = 16384;
int linMax = 250;

struct {
	float ang;
	uint16_t lin;
	uint16_t hold;
} send_params;

union UN {
	struct {
		float lin;
		float ang;
		int hold;
	} params;
	uint8_t bytes[sizeof(params)];
} un, un_send;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void StartAMT22Data(void const * argument);
void StartUARTData(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t cntImpulse1 = 0, cntImpulse2 = 0, step1 = 0, step2 = 0;

//RoboArm arm(120, 124);
RoboArm arm(0, 124);

//Эталонно работающая функция - удалить
int steppingyakkazavmaxim(float stepM1, float stepM2) {

	arm.anglePsteps = stepM1;
	arm.distPsteps = stepM2;

	//числа 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);

	//  частота шим = входящая частота / период (arr)
	//  125 000 (125 килогерц)  = 16 000 000 / 128
	// (1/60)*1000 = частота 16 (герц);
	float periodM1 = 1200; //мікросекунд
	uint32_t psc = 72;

	if (stepM1 > stepM2) {

		htim1.Instance->PSC = psc;
		htim1.Instance->ARR = periodM1;
		htim1.Instance->CCR1 = periodM1 / 2;

		float delimiter = stepM1 / stepM2;
		float mnoj = ceil(periodM1 * delimiter);

		htim2.Instance->PSC = psc;
		htim2.Instance->ARR = mnoj;
		htim2.Instance->CCR2 = mnoj / 2;

	} else if (stepM1 < stepM2) {
//	uint8_t delimiter=stepM2/stepM1;
//	uint16_t impMore = (72000000/psc_max)/1000; 						//імпульсів кроків за секунду для мотора з більшої кількістю кроків  КРОКІВ НА СЕКУНДУ
//	uint16_t allSecMore = (stepM2/impMore)*1000;		 				//загальний час роботи мотора із більшої кількістю кроків  мілісекунд
//	uint16_t stepSecM1 =  (stepM1/allSecMore)*1000; 					//кроків на секунду на двигуна LESS  250
//	uint16_t PSCmLess= 72000000 / (stepSecM1 * 1000); 					//дільник для мотора LESS

		htim2.Instance->PSC = psc;
		htim2.Instance->ARR = periodM1;
		htim2.Instance->CCR2 = periodM1 / 2;

		float delimiter = stepM2 / stepM1;
		float mnoj = ceil(periodM1 * delimiter);

		htim1.Instance->PSC = psc;
		htim1.Instance->ARR = mnoj;
		htim1.Instance->CCR1 = mnoj / 2;

	}

	//Старт таймера та переривань

	arm.SetEnable(1, true);
	arm.SetEnable(2, true);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	return 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Init(&htim1);
	HAL_TIM_Base_Init(&htim2);

	arm.SetSettMotors(huart2,htim1, htim2, htim3, Dir1_GPIO_Port, Dir1_Pin,
			Dir2_GPIO_Port, Dir2_Pin, Dir3_GPIO_Port, Dir3_Pin, En1_GPIO_Port,
			En1_Pin, En2_GPIO_Port, En2_Pin, En3_GPIO_Port, En3_Pin);

	arm.SetSettEncoders(hspi1, CS1_GPIO_Port, CS1_Pin, CS2_GPIO_Port, CS2_Pin,
			14);
//	arm.SetMicrosteps(8);
//	arm.SetSoftwareZero();
//	arm.SetZeroEncoders();

	//steppingyakkazavmaxim(15000, 12000);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myAMT22Task */
  osThreadDef(myAMT22Task, StartAMT22Data, osPriorityIdle, 0, 128);
  myAMT22TaskHandle = osThreadCreate(osThread(myAMT22Task), NULL);

  /* definition and creation of myUARTTask */
  osThreadDef(myUARTTask, StartUARTData, osPriorityIdle, 0, 128);
  myUARTTaskHandle = osThreadCreate(osThread(myUARTTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
//
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
//
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
//
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, En2_Pin|En1_Pin|Dir1_Pin|EndCap4_Pin
                          |Buser_Pin|Reserved_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS2_Pin|Dir2_Pin|Led1_Pin
                          |EndCap1_Pin|EndCap2_Pin|S1_Pin|S2_Pin
                          |EndCap3_Pin|En3_Pin|Dir3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : En2_Pin En1_Pin Dir1_Pin EndCap4_Pin
                           Buser_Pin Reserved_Pin */
  GPIO_InitStruct.Pin = En2_Pin|En1_Pin|Dir1_Pin|EndCap4_Pin
                          |Buser_Pin|Reserved_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS2_Pin Dir2_Pin Led1_Pin
                           EndCap1_Pin EndCap2_Pin S1_Pin S2_Pin
                           EndCap3_Pin En3_Pin Dir3_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin|Dir2_Pin|Led1_Pin
                          |EndCap1_Pin|EndCap2_Pin|S1_Pin|S2_Pin
                          |EndCap3_Pin|En3_Pin|Dir3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//UNUSED(huart);
	if (huart == &huart1) {
//		if(!strcmp(rx_buffer,"TEXT")) {
//		}

//		uint8_t data[] = { '\\', 0x8f, 0xf8, 'B', 'q', '}', 0x16, 'C', 1, 1, 0, 0 };
		memcpy(un.bytes, rx_buffer, sizeof(rx_buffer));
		switch (un.params.hold) {
		case 0:
		case 1:
			startFirstMove = true;
			break;
		case 25:
			stopBeforeReboot = true;
		case 50:
			sendDataFlag = true;
			break;
		case 75:
			setZeroFlag = true;
		}
		memset(rx_buffer, 0, sizeof(rx_buffer));
		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	}
	HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	arm.setPrintState(false);
	/* Infinite loop */
	for (;;) {

		if (startFirstMove) {
			startFirstMove = false;
			float angle = arm.ShiftZeroInputAng(un.params.ang);
			uint16_t distance = arm.ShiftZeroInputLin(un.params.lin);
			arm.Move2MotorsSimu(angle, distance);
//			arm.correctPosition();
//			arm.Move2MotorsSimu(un.params.ang, un.params.lin);
//			arm.Move2MotorsSimu(recAngleF, recDist);
			//steppingyakkazavmaxim(2000, 230);
		}

		if (timerFT1 && timerFT2) {

			timerFT1 = false;
			timerFT2 = false;

			correctPosFlag = false;
			arm.correctPosition();

		}

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAMT22Data */
/**
 * @brief Function implementing the myAMT22Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAMT22Data */
void StartAMT22Data(void const * argument)
{
  /* USER CODE BEGIN StartAMT22Data */

	/* Infinite loop */
	for (;;) {

		osDelay(50);
	}
  /* USER CODE END StartAMT22Data */
}

/* USER CODE BEGIN Header_StartUARTData */
/**
 * @brief Function implementing the myUARTTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUARTData */
void StartUARTData(void const * argument)
{
  /* USER CODE BEGIN StartUARTData */
	HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
	uint32_t posnowT_1, posnowT_2;
//	float angleT = 0;
//	uint32_t linearDist = 0;
//	flagReadEnc = 1;
//	uint32_t distPmm = 0;
	arm.setPrintState(true);

	/* Infinite loop */
	for (;;) {

		if (arm.getPrintState() && sendDataFlag) {
			sendDataFlag = false;

			int attempts = 0;

			posnowT_1 = arm.GetPosEncoders(1);
//			//if the position returned was 0xFFFF we know that there was an error calculating the checksum
			//make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
			while (posnowT_1 == 0xFFFF && ++attempts < 3)
				posnowT_1 = arm.GetPosEncoders(1); //try again

//			float ang = posnowT_1*360/16384;
			float ang_actual = arm.GetAngleEncoders(posnowT_1);
			float ang = arm.ShiftZeroOutputAng(ang_actual);

			attempts = 0;
//			un_send.params.ang = angleT;

			posnowT_2 = arm.GetPosEncoders(2);
			while (posnowT_2 == 0xFFFF && ++attempts < 3)
				posnowT_2 = arm.GetPosEncoders(2); //try again

			float ang_pos = arm.GetAngleEncoders(posnowT_2);
			float pos_actual = ang_pos * 250.0 / 360.0;
			float pos = arm.ShiftZeroOutputLin(pos_actual);
//			angleT = arm.GetAngleEncoders(posnowT) * 100;
//
//			float distPsteps = angleT * (motorStep * drvMicroSteps)
//					* (6.45 / 360);
//			uint32_t mils = distPsteps / arm.linearStepsMil;
//
//			un_send.params.lin = mils;
//			un_send.params.hold = 0;
			un_send.params.lin = pos;
			un_send.params.ang = ang;
			un_send.params.hold = 0;

//			uint8_t send_arr[] = {0x01,0x02,0x03,0x04,
//					0x05,0x06,0x07,0x08,
//					0x09,0x0a,0x0b,0x0c};
//
//			size_t s = sizeof(un_send);

//			sprintf(str, "%.2f:", un_send.params.ang);//, send_params.lin, 100);
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);

//			HAL_UART_Transmit(&huart1, (uint8_t*)str, sizeof(str), 12);
//			sendDataFlag = false;

//			char str[100];
//			posnowT = arm.GetPosEncoders(1);
//			angleT = arm.GetAngleEncoders(posnowT) * 100;
//			sprintf(str, "x: enc: %d | ang: %d \n", posnowT, (uint16_t) angleT);
//			HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str),
//					HAL_MAX_DELAY);
//
//			posnowT = arm.GetPosEncoders(2);
//			angleT = arm.GetAngleEncoders(posnowT) * 100;
//
//			float distPsteps = angleT * (motorStep * drvMicroSteps)
//					* (6.45 / 360);
//			uint32_t mils = distPsteps / arm.linearStepsMil;
//
//			sprintf(str, "y: enc: %d | ang: %d | mm: %d \n", posnowT,
//					(uint16_t) angleT, mils);
//
//			HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str),
//					HAL_MAX_DELAY);
		}

		if (stopBeforeReboot) {
			stopBeforeReboot = false;
			arm.EmergencyStop();
			un_send.params.lin = 0;
			un_send.params.ang = 0;
			un_send.params.hold = 10;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);
		}

		if (setZeroFlag) {
			setZeroFlag = false;
//			arm.SetZeroEncoders();
			arm.SetSoftwareZero();
			un_send.params.lin = 0;
			un_send.params.ang = 0;
			un_send.params.hold = 10;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);
		}

		osDelay(500);
	}
  /* USER CODE END StartUARTData */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	if (htim->Instance == TIM1)/*Проверяем от какого таймера пришёл CallBack тут надо проверить точность*/
	{
		cntImpulse1++;
		if (cntImpulse1 >= arm.anglePsteps) {

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim1);
			arm.SetEnable(1, false);
			cntImpulse1 = 0;
			arm.stateMoveM1 = false;
			timerFT1 = true;
		}

	} else if (htim->Instance == TIM2) {

		cntImpulse2++;
		if (cntImpulse2 >= arm.distPsteps) {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim2);
			arm.SetEnable(2, false);
			cntImpulse2 = 0;
			arm.stateMoveM2 = false;
			timerFT2 = true;
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
	__disable_irq();
	while (1) {
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
