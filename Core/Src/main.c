/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "iks01a3_motion_sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH 5 //number of samples used by the mobile avg filter
#define MIC_MIN 127 //min level of microphone
#define MIC_MAX 256 //max level of microphone
#define TELEPORT_DELAY 42000 //Delay between two presses of teleport (so that the teleport is not spammed)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile _Bool TIM3_FLAG = 0, TIM4_FLAG = 0, EOC_FLAG = 0, correctlySentData = 1;//flags used for the callbacks
volatile int n_ovf = 0;
int32_t pitch_vector[LENGTH], roll_vector[LENGTH]; //vectors containing the last n (n = LENGTH) samples of pitch and roll angles
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static int Update_pitch_vector(int data);
static int Update_roll_vector(int data);
static void Initialize_vectors();
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
	float pitch = 0, roll = 0;
	char printData[32]; // message to be sent
	IKS01A3_MOTION_SENSOR_Axes_t axes; // to contain the output of the sensor
	int32_t filtered_pitch = 0, filtered_roll = 0;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

    /*Hardware start*/
  	HAL_ADC_Start_IT(&hadc1); // Start Analog to Digital Converter
	HAL_TIM_Base_Start_IT(&htim3); // Start TIM3 to send a periodic interrupt of 240 Hz
	HAL_TIM_Base_Start_IT(&htim4); // Start TIM4 to limit the teleport frequency to 2 Hz
	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,34);

	IKS01A3_MOTION_SENSOR_Init(1, MOTION_ACCELERO);
	IKS01A3_MOTION_SENSOR_Enable(1, MOTION_ACCELERO);

	/*Initialization of roll and pitch vectors at 0*/
	Initialize_vectors();
	int32_t time_spent = 0;
	int32_t begin = 0, end = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		n_ovf = 0;
		begin = __HAL_TIM_GetCounter(&htim3);

		if(TIM3_FLAG) { // this flag is set 240 times a second
			TIM3_FLAG = 0;
			sprintf(printData,"\r\n");
			if(!IKS01A3_MOTION_SENSOR_GetAxes(1, MOTION_ACCELERO, &axes)) { // if the sensor reading is successful

				// pitch computation and filtering : RIGHT - LEFT (a and d button of the keyboard)
				pitch = atan(-1 * axes.y / sqrt(pow(axes.x, 2) + pow(axes.z, 2))) * 12;
				filtered_pitch = filtered_pitch + pitch - Update_pitch_vector(pitch);
				if(filtered_pitch > 20) {
					strcat(printData, "d");
				}
				else if(filtered_pitch < -20) {
					strcat(printData, "a");
				}

				// roll computation and filtering - TELEPORT (s button of the keyboard)
				roll = atan(-1 * axes.x / sqrt(pow(axes.y, 2) + pow(axes.z, 2))) * 12;
				filtered_roll = filtered_roll + roll - Update_roll_vector(roll);
				if (filtered_roll < 0) {
					if(TIM4_FLAG){
						// if half a second has passed since the last fired shot
						TIM4_FLAG = 0;
						strcat(printData, "s");
						__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+TELEPORT_DELAY); // half a second needs to pass before the next shot
					}
				}
			}
			// blue button reading : FIRING (space button of the keyboard)
			if (!HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)) {
				strcat(printData, " ");
			}

			// microphone reading : FORWARD (w button of the keyboard)
			if (EOC_FLAG == 1) {
				if (HAL_ADC_GetValue(&hadc1) > MIC_MIN && HAL_ADC_GetValue(&hadc1) < MIC_MAX) {
					if(HAL_ADC_GetValue(&hadc1)> 230) {
						strcat(printData, "w");
						EOC_FLAG = 0;
					}
				}
			}
		}

		//transmits the letters corresponding to the buttons of the keyboard that have to be pressed
		HAL_UART_Transmit_IT(&huart2,(uint8_t *)printData, strlen(printData));
		if(correctlySentData) HAL_UART_Transmit_IT(&huart2, (uint8_t *)printData, strlen(printData));
		correctlySentData = 0;

		end = __HAL_TIM_GetCounter(&htim3);
		time_spent = end - begin + n_ovf*65535;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_CC1;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 3499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*Callback override of tim3, tim4, ADC and UART*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		n_ovf++;
	}
}

/**
  * @brief  Timer OC callback override to set the flags that allow to capture signals and send letters at precise time instants
  * @param  htim : it's the variable corresponding to the timers in use
  * @retval none
  */
void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef *htim){
	if (htim == &htim3) {
		TIM3_FLAG = 1; //240Hz flag to write the chars " ","a","d" in printData (see the while)
	}
	if (htim == &htim4) {
		TIM4_FLAG = 1; //2400Hz or 2Hz (after having pressed "s") flag to write the char "s" in printData (see the while)
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+34);
	}
}

/**
  * @brief  ADC callback override to capture the audio signal coming from the microphone
  * @param  hadc : it's the variable corresponding to the adc in use
  * @retval none
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc == &hadc1){
		EOC_FLAG = 1; //flag to communicate end of conversion of the ADC. This is used to write the char "w" in printData (see the while)
		__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
	}
}

/**
  * @brief  UART callback override used to set the flag that allows to send strings through the COM port
  * @param  huart : it's the variable corresponding to the adc in use
  * @retval none
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	correctlySentData = 1;
}

/*

/**
  * @brief  This function initializes the vectors to zero
  * @param  none
  * @retval none
  */
static void Initialize_vectors() {
	int i;
	for(i = 0; i < LENGTH; i++) {
		pitch_vector[i] = 0;
		roll_vector[i] = 0;
	}
}

/**
  * @brief  This function pushes the new sample of pitch inside the vector with its
  * 	    last n = LENGTH sample and returns the discarded value
  * @param  data : new measurement of pitch
  * @retval first_element: first element of pitch_vector
  */
static int Update_pitch_vector(int data) {
	int i;
	int first_element = pitch_vector[0];
	for (i = 0; i < LENGTH - 1; i++) {
		pitch_vector[i] = pitch_vector[i + 1];
	}
	pitch_vector[LENGTH - 1] = data;
	return first_element;
}

/**
  * @brief  This function pushes the new sample of pitch inside the vector with its
  * 	    last n = LENGTH sample and returns the discarded value
  * @param  data : new measurement of roll
  * @retval first_element: first element of roll_vector
  */
static int Update_roll_vector(int data) {
	int i;
	int first_element = roll_vector[0];
	for (i = 0; i < LENGTH - 1; i++) {
		roll_vector[i] = roll_vector[i + 1];
	}
	roll_vector[LENGTH - 1] = data;
	return first_element;
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

