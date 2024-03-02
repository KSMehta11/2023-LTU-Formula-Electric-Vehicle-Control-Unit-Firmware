/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Kaushik Mehta
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

#include "CAN.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{

	true = 1,
	false = 0

} bool;

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

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

osThreadId TorqueControlHandle;
osThreadId NodeGuardingHandle;
osThreadId OrionDataHandle;
osThreadId PWMHandle;
osThreadId OrionDerateDataHandle;
osThreadId IgnitionHandle;
/* USER CODE BEGIN PV */

// Ignition
GPIO_PinState buttonState, lastButtonState = GPIO_PIN_SET;
bool enableCar = false;
unsigned long buttonPressTime = 0;

// Task times update according to clock speed
unsigned long previousTaskTime[8] = { 0 };
const uint8_t BMS_CYCLE_TIME = 60;
const uint8_t TORQUE_CYCLE_TIME = 10;
const uint8_t NODE_GUARDING_CYCLE_TIME = 70;
const uint8_t TIMEOUT = 20;


// Error Detection
bool errorSet[3] = { false };

// Torque Control
uint32_t primaryPotValue = 0, secondaryPotValue = 0;
uint32_t buffer1 = 0, buffer2 = 0;
int torqueRefLimit = 5000;
const int SPEED_REF_LIMIT =	(6500 + 32768);
const int THROTTLE_POSITION_MAX_1 = 234;
const int THROTTLE_POSITION_MIN_1 = 300;
const int THROTTLE_POSITION_MAX_2 = 382;
const int THROTTLE_POSITION_MIN_2 = 315;
const int TORQUE_REF_LIM_MAX = 9500;
const int TORQUE_REF_LIM_MIN = 5000;

// CAN Rx Header FIFO0
CAN_RxHeaderTypeDef rxHeaderFIFO0;
uint8_t dataFIFO0[8] = { 0 };

// CAN Rx Header FIFO1
CAN_RxHeaderTypeDef rxHeaderFIFO1;
uint8_t data[8] = { 0 };

// BMS Data
float packStateOfCharge = 0;
uint16_t packCurrent = 0;

// RPM Data
int rpm = 0;

// Temperature Data
int motorTemp = 0;
int motorControllerTemp = 0;

// PWM
int pumpPWM = 0;
int fanPWM = 0;

// Brake Pressure Sensor
uint16_t brakePressure = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC3_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTask06(void const * argument);

/* USER CODE BEGIN PFP */

// Pedal Position Output Filter (Eliminates induces noise and spikes in signal)
uint32_t ADC_GetAverageValue(ADC_HandleTypeDef* hadc, int numOfReadings)
{

	int max = 0, min = 0;
	int maxIndex = 0, minIndex = 0;
	uint32_t data[10] = { 0 };
	uint32_t sum = 0, average = 0;

	for (int i = 0; i < numOfReadings; i++)
	{
		HAL_ADC_Start(hadc);

		HAL_ADC_PollForConversion(hadc, 1);

		data[i] = HAL_ADC_GetValue(hadc);

		HAL_ADC_Stop(hadc);
	}

	max = data[0];

	for (int i = 1; i < 10; i++)
	{
		if (data[i] > max)
		{
			max = data[i];
			maxIndex = i;
		}
	}

	min = data[0];

	for (int i = 1; i < 10; i++)
	{
		if (data[i] < min)
		{
			min = data[i];
			minIndex = i;
		}
	}

	for (int i = 0; i < 10; i++)
	{
		if (i == minIndex || i == maxIndex)
		{

		}
		else
		{
			sum += data[i];
		}
	}

	average = sum / 8;

	data[maxIndex] = average;

	data[minIndex] = average;

	sum = 0;

	for (int i = 0; i < 10; i++)
	{
		sum += data[i];
	}

	average = sum / 10;

	return (average);
}
/*
void ignitionTask()
{
	buttonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

	if (lastState == GPIO_PIN_SET && buttonState == GPIO_PIN_RESET)
	{
		buttonPressTime = HAL_GetTick();
	}
	else if (lastState == GPIO_PIN_RESET && buttonState == GPIO_PIN_SET)
	{
		buttonReleaseTime = HAL_GetTick();

		long pressDuration = buttonReleaseTime - buttonPressTime;

		if (pressDuration > 500)
		{
			startNode();
			clearErrors();

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

			TIM2->CNT = 0;
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

			enableCar = true;
		}
	}

	lastState = buttonState;
}
*/

// Handles Vehicle Ignition Process
void ignitionTask()
{
	buttonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

	buttonPressTime = HAL_GetTick();

	while (buttonState == GPIO_PIN_RESET)
	{
		if (HAL_GetTick() - buttonPressTime > 50)
		{
			startNode();
			clearErrors();

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

			TIM2->CNT = 0;
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

			enableCar = true;
		}

		buttonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	}
}

bool timeCheck(int duration, int taskNum)
{
	unsigned long currentTaskTime = HAL_GetTick();

	if (currentTaskTime - previousTaskTime[taskNum] > duration)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// General purpose map function
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// General constrain function
long constrain(long x, long out_min, long out_max)
{
	if (x < out_min)
	{
		x = out_min;
	}

	if (x > out_max)
	{
		x = out_max;
	}

	return x;
}

// Primary Pedal position sensor range check
void primaryPotErrorCheck()
{
	primaryPotValue = constrain(ADC_GetAverageValue(&hadc1, 10), 0, 1023);

	if (primaryPotValue <= 0 || primaryPotValue >= 550)
	{
		errorSet[0] = true;
	}
	else
	{
		errorSet[0] = false;
	}
}

// Secondary Pedal position sensor range check
void secondaryPotErrorCheck()
{
	secondaryPotValue = constrain(ADC_GetAverageValue(&hadc2, 10), 0, 1023);

	if (secondaryPotValue <= 0 || secondaryPotValue >= 550)
	{
		errorSet[1] = true;
	}
	else
	{
		errorSet[1] = false;
	}
}

// Brake pressure sensor range check
void brakeSensorErrorCheck()
{
	brakePressure = ADC_GetAverageValue(&hadc3, 10);

	brakePressure += 44;

	if (brakePressure <= 90 || brakePressure >= 260)
	{
		errorSet[2] = true;
	}
	else
	{
		errorSet[2] = false;
	}
}

// Torque Control and Regenerative Braking Function
void torqueControl()
{

	if (errorSet[0] == true && errorSet[1] == true)
	{
		torqueRefLimit = 5000;
	}
	else if (errorSet[0] == true)
	{
		secondaryPotValue = constrain(ADC_GetAverageValue(&hadc2, 10), 0, 1023);

		torqueRefLimit = map(secondaryPotValue, THROTTLE_POSITION_MIN_2, THROTTLE_POSITION_MAX_2, TORQUE_REF_LIM_MIN, TORQUE_REF_LIM_MAX);

		torqueRefLimit = constrain(torqueRefLimit, TORQUE_REF_LIM_MIN, TORQUE_REF_LIM_MAX);
	}
	else
	{
		primaryPotValue = constrain(ADC_GetAverageValue(&hadc1, 10), 0, 1023);

		torqueRefLimit = map(primaryPotValue, THROTTLE_POSITION_MIN_1, THROTTLE_POSITION_MAX_1, TORQUE_REF_LIM_MIN, TORQUE_REF_LIM_MAX);

		torqueRefLimit = constrain(torqueRefLimit, TORQUE_REF_LIM_MIN, TORQUE_REF_LIM_MAX);
	}

	brakePressure = ADC_GetAverageValue(&hadc3, 10);

    brakePressure += 44;

	if (brakePressure > 110 && brakePressure < 127)
	{
		 if (rpm > 200 && errorSet[0] == false && errorSet[1] == false && errorSet[2] == false)
		 {
			 torqueRefLimit = map(torqueRefLimit, 110, 127, 5000, 4500);
		 }
	}

	torqueControlMessage(&torqueRefLimit, &SPEED_REF_LIMIT);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// RTD Speaker interrupt handler
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_OC_DelayElapsedCallback could be implemented in the user file
   */
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

  //vTraceEnable(TRC_START);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  // Iniitialize CAN Protocol
  HAL_CAN_Start(&hcan1);

  // Pump and Fan PWM
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  TIM8->CCR3 = 50;

  TIM8->CCR4 = 50;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

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
  /* definition and creation of TorqueControl */
  osThreadDef(TorqueControl, StartDefaultTask, osPriorityNormal, 0, 200);
  TorqueControlHandle = osThreadCreate(osThread(TorqueControl), NULL);

  /* definition and creation of NodeGuarding */
  osThreadDef(NodeGuarding, StartTask02, osPriorityAboveNormal, 0, 200);
  NodeGuardingHandle = osThreadCreate(osThread(NodeGuarding), NULL);

  /* definition and creation of OrionData */
  osThreadDef(OrionData, StartTask04, osPriorityNormal, 0, 200);
  OrionDataHandle = osThreadCreate(osThread(OrionData), NULL);

  /* definition and creation of PWM */
  osThreadDef(PWM, StartTask05, osPriorityNormal, 0, 200);
  PWMHandle = osThreadCreate(osThread(PWM), NULL);

  /* definition and creation of OrionDerateData */
  osThreadDef(OrionDerateData, StartTask06, osPriorityNormal, 0, 200);
  OrionDerateDataHandle = osThreadCreate(osThread(OrionDerateData), NULL);

  /* definition and creation of Ignition */
  osThreadDef(Ignition, StartTask06, osPriorityNormal, 0, 200);
  IgnitionHandle = osThreadCreate(osThread(Ignition), NULL);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  motorAndControllerTempFilterConfig();

  motorRPMFilterConfig();

  orionFilterConfig();

  //orionFilterConfig_1();

  /* USER CODE END CAN1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 18000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 36-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC5 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument) // Torque Control Thread
{
  /* USER CODE BEGIN 5 */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

	  vTaskDelayUntil(&xLastWakeTime, xFrequency);

	  syncMessage();

	  vTaskDelay(pdMS_TO_TICKS(2));

	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rxHeaderFIFO1, data);

	  if (rxHeaderFIFO1.StdId == 0x3FA)
	  {
		  rpm = ((int)(data[1]<<8) | (int)data[0]) - 32768;
	  }

	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rxHeaderFIFO1, data);

	  if (rxHeaderFIFO1.StdId == 0x1BA)
	  {
		  motorControllerTemp = (int)data[0] - 40;

		  motorTemp = (int)data[1] - 40;
	  }

	  primaryPotErrorCheck();

	  secondaryPotErrorCheck();

	  torqueControl();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the NodeGuarding thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument) // Inverter Node Guarding Thread
{
  /* USER CODE BEGIN StartTask02 */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 150;

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);

	  nodeGuarding();
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the DataAquisition thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument) // BMS Torque Deration Thread
{
  /* USER CODE BEGIN StartTask04 */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 95;

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);

	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeaderFIFO0, dataFIFO0);

	  if (rxHeaderFIFO0.StdId == 0x6CA)
	  {
		  packCurrent = ((dataFIFO0[0]<<8) | dataFIFO0[1]) / 100;

		  packStateOfCharge = ((dataFIFO0[2]<<8) | dataFIFO0[3]) / 100.0 * 2.0;
	  }

  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the PWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument) // Radiator Fan and Pump PWM Thread
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {

	  if (motorTemp > 30 || motorControllerTemp > 30)
	  {
		  if (motorTemp > motorControllerTemp)
		  {
			  constrain(motorTemp, 25, 80);

			  pumpPWM = constrain(map(motorTemp, 25, 80, 30, 50), 30, 50);

			  TIM8->CCR3 =  pumpPWM; // Pump

			  fanPWM = constrain(map(motorTemp, 25, 80, 30, 50), 30, 50);

			  TIM8->CCR4 = fanPWM; // Fan
		  }
		  else
		  {
			  constrain(motorControllerTemp, 25, 55);

			  pumpPWM = constrain(map(motorControllerTemp, 25, 55, 30, 50), 30, 50);

			  TIM8->CCR3 = pumpPWM; // Pump

			  fanPWM = constrain(map(motorControllerTemp, 25, 55, 30, 50), 30, 50);

			  TIM8->CCR4 = fanPWM; // Fan
		  }
	  }
	  else
	  {
		  pumpPWM = 50;

		  fanPWM = 50;

		  TIM8->CCR3 = pumpPWM;

		  TIM8->CCR4 = fanPWM;
	  }

  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the OrionDerateData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument) // Ignition Thread
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
	  ignitionTask();
  }
  /* USER CODE END StartTask06 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
  while (1)
  {
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
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
