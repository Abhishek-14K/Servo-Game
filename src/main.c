/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "mfs.h"
#include "calibration.h"
#include "stm32l476xx.h"
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
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for controller */
osThreadId_t controllerHandle;
const osThreadAttr_t controller_attributes = {
  .name = "controller",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servo1 */
osThreadId_t servo1Handle;
const osThreadAttr_t servo1_attributes = {
  .name = "servo1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servo2 */
osThreadId_t servo2Handle;
const osThreadAttr_t servo2_attributes = {
  .name = "servo2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
int min_sig, max_sig, new_round, roun, match, game_end, game_start, current_round, firstround;
int serv1_sig;
int button1flag, button2flag, button3flag, checkflag;
int cal1_sig;
char buffer[200];
unsigned int time, timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartController(void *argument);
void StartServo1(void *argument);
void StartServo2(void *argument);

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
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Starting PWM at TIM2 CH1
 // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Starting PWM at TIM2 CH2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Starting PWM at TIM2 CH4

  min_sig = cal1(); // Calibrating servo to get minimum signal
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\rMinimum:%d",min_sig), 500);
  max_sig = cal2(); // Calibrating servo to get maximum signal
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\rMaximum:%d",max_sig), 500);

  TIM2->CCR1 = min_sig; // Moving servo to starting position
  TIM2->CCR4 = min_sig; // Moving servo to starting position
  cal1_sig = min_sig; // Assigning value of minimum to signal to variable

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of controller */
  controllerHandle = osThreadNew(StartController, NULL, &controller_attributes);

  /* creation of servo1 */
  servo1Handle = osThreadNew(StartServo1, NULL, &servo1_attributes);

  /* creation of servo2 */
  servo2Handle = osThreadNew(StartServo2, NULL, &servo2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  htim2.Init.Prescaler = 320-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

		  /* GPIO Ports Clock Enable */
		  __HAL_RCC_GPIOC_CLK_ENABLE();
		  __HAL_RCC_GPIOH_CLK_ENABLE();
		  __HAL_RCC_GPIOA_CLK_ENABLE();
		  __HAL_RCC_GPIOB_CLK_ENABLE();

		  /*Configure GPIO pin Output Level */
		  HAL_GPIO_WritePin(GPIOA, SHLD_D13_Pin|SHLD_D12_Pin|SHLD_D11_Pin|SHLD_D7_SEG7_Clock_Pin
		                          |SHLD_D8_SEG7_Data_Pin, GPIO_PIN_SET);

		  /*Configure GPIO pin Output Level */
		  HAL_GPIO_WritePin(GPIOB, SHLD_D4_SEG7_Latch_Pin|SHLD_D10_Pin, GPIO_PIN_SET);

		  /*Configure GPIO pin : B1_Pin */
		  GPIO_InitStruct.Pin = B1_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_A5_Pin SHLD_A4_Pin */
		  GPIO_InitStruct.Pin = SHLD_A5_Pin|SHLD_A4_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_A0_Pin SHLD_D2_Pin */
		  GPIO_InitStruct.Pin = SHLD_A0_Pin|SHLD_D2_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_A1_Pin SHLD_A2_Pin */

		  GPIO_InitStruct.Pin = SHLD_A1_Pin|SHLD_A2_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
		  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_D13_Pin SHLD_D12_Pin SHLD_D11_Pin SHLD_D7_SEG7_Clock_Pin */
		  GPIO_InitStruct.Pin = SHLD_D13_Pin|SHLD_D12_Pin|SHLD_D11_Pin|SHLD_D7_SEG7_Clock_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_A1_Pin */
		//  GPIO_InitStruct.Pin = SHLD_A1_Pin;
		 // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		 // GPIO_InitStruct.Pull = GPIO_PULLUP;
		  //HAL_GPIO_Init(SHLD_A1_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_A2_Pin */
		 // GPIO_InitStruct.Pin = SHLD_A2_Pin;
		 // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  //GPIO_InitStruct.Pull = GPIO_PULLUP;
		  //HAL_GPIO_Init(SHLD_A2_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_A3_Pin */
		  GPIO_InitStruct.Pin = SHLD_A3_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  HAL_GPIO_Init(SHLD_A3_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_D6_Pin SHLD_D5_Pin */
		  GPIO_InitStruct.Pin = SHLD_D6_Pin|SHLD_D5_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D9_Pin */
		  GPIO_InitStruct.Pin = SHLD_D9_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(SHLD_D9_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D8_SEG7_Data_Pin */
		  GPIO_InitStruct.Pin = SHLD_D8_SEG7_Data_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		  HAL_GPIO_Init(SHLD_D8_SEG7_Data_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D4_SEG7_Latch_Pin */
		  GPIO_InitStruct.Pin = SHLD_D4_SEG7_Latch_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		  HAL_GPIO_Init(SHLD_D4_SEG7_Latch_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D10_Pin */
		  GPIO_InitStruct.Pin = SHLD_D10_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(SHLD_D10_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_D15_Pin SHLD_D14_Pin */
		  GPIO_InitStruct.Pin = SHLD_D15_Pin|SHLD_D14_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartController */
/**
  * @brief  Function implementing the controller thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartController */
void StartController(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_RESET && button2flag == 0) // If button pressed
	  {
		 // HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",3), 500);
		  roun = 0; // Set number of rounds to zero
		  checkflag = 0; // Initialize flag to check for matching signals to zero
		  firstround = 0; // Initialize flag for first round to zero
		  game_start = 1; // Initialize game start flag to 1
		  game_end = 0;
		  time = 0; // Initialize counter value of total time to zero
		  timer = 0; // Initialize counter value of time when servo 1 moves to be zero
		 // __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
		  HAL_TIM_Base_Start(&htim3); // Start timer
		  TIM2->CCR1 = min_sig; // Move both servos to first position
		  TIM2->CCR4 = min_sig;
		  cal1_sig = min_sig;
		  button2flag = 1; // Flag to check button press
		  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\rGame Start!"), 500);
	  }

	  else if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_SET) // If button released
	  {
		  if (button2flag == 1)
		  {

			  button2flag = 0; // Flag to check button release
		  }
	  }

	  if (roun < 5 && game_start == 1) // If game in ongoing
	  {
		  if (firstround == 0) // Check if first round started
		  {
			  firstround = 1; // Flag to check off first round starting
			  new_round = 1; // Flag to start new new round
		  }

	  }

	  if (new_round == 1 && roun < 5) // Check if new round can be started
	  {
			int	delay = (HAL_RNG_GetRandomNumber(&hrng) % 4) + 1; // Random delay between 1 to 4 seconds

			osDelay(delay*1000);
			timer = __HAL_TIM_GET_COUNTER(&htim3);//HAL_GetTick(); // Get timer value
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",timer), 500);
					//__HAL_TIM_GET_COUNTER(&htim3);
			int serv = (HAL_RNG_GetRandomNumber(&hrng) % 6); // Move to random positions between 0 and 5
			serv1_sig = min_sig+(serv*20);
			TIM2->CCR4 = serv1_sig; // Move servo to position
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%ld",TIM2->CCR4), 500);

			roun++; // Increment round value
			new_round = 0; // Flag to show round is completed
			checkflag = 0; // Set flag to chheck for signal match to zero
	  }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartServo1 */
/**
* @brief Function implementing the servo1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServo1 */
void StartServo1(void *argument)
{
  /* USER CODE BEGIN StartServo1 */
  /* Infinite loop */
  for(;;)
  {

		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_RESET && button1flag == 0) // Button 1 is pressed
			  {

				  //cal1_sig + 30;
				  cal1_sig -= 20; // Decrement signal value
				  TIM2->CCR1 = cal1_sig; // Move servo to position
				  //cal1_sig = TIM2->CCR1;
				  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",1), 500);
				  button1flag = 1; // Check button flag
			  }



		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_SET) // Button is released
		  {

			  button1flag = 0;
		  }

		  if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_RESET && button3flag == 0) // Button 3 is pressed
		  {

			  cal1_sig += 20; // Increment signal value
			  TIM2->CCR1 = cal1_sig; // Move servo to position
			  //cal1_sig = TIM2->CCR1;
			  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",2), 500);
			  button3flag = 1; // Check button flag

		  }

		  if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_SET) // Button is released
		  {

			  button3flag = 0;

		  }

		  if( TIM2->CCR1 == serv1_sig && checkflag == 0) // Check if signals of both servos match and check flag is zero
		  {
			  checkflag = 1; // Flag to let us know a check happened
			  int current_time = __HAL_TIM_GET_COUNTER(&htim3);//HAL_GetTick(); // Get current time
					  //__HAL_TIM_GET_COUNTER(&htim3);
			  time += (current_time-timer); // add to total accumulated time
			  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\rMatch:%u",current_time-timer), 500);
			  new_round = 1; // Set flag to start new round
			  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\rMatch!!",2), 500);


		  }

		  if (time > 9999) // If time is in 5 digits
		  {
			  segdisplay(time/1000); // Display is seconds
		  }
		  else{
		  segdisplay(time); // Display time in ms on 7 seg display
		  }



  }
  /* USER CODE END StartServo1 */
}

/* USER CODE BEGIN Header_StartServo2 */
/**
* @brief Function implementing the servo2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServo2 */
void StartServo2(void *argument)
{
  /* USER CODE BEGIN StartServo2 */
  /* Infinite loop */
  for(;;)
  {
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\rhere1"), 500);


  }
  /* USER CODE END StartServo2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

