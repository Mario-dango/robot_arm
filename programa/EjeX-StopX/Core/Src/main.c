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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

 uint32_t velMotor1 = 0;
 uint16_t thetaMotor1 = 0;
 // uint32_t adcVal;					// Arreglo para guardar los valores leidos del conversor ADC

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);		// Función para el manejo de interrupciones del Timer 2
// void EXTI9_5_IRQHandler(void);		// Función para manejo de la interrupciones externas por fin de carrera
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, StepM1_Pin);
  HAL_TIM_Base_Start_IT(&htim2);			// Iniciar el temporizador con interrupción

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
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
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedPcb_GPIO_Port, LedPcb_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DirM_X_GPIO_Port, DirM_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ResetMotors_Pin|SleepMotors_Pin|EnableMotors_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LedPcb_Pin */
  GPIO_InitStruct.Pin = LedPcb_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedPcb_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DirM_X_Pin */
  GPIO_InitStruct.Pin = DirM_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DirM_X_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : StopM_X_Pin */
  GPIO_InitStruct.Pin = StopM_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(StopM_X_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ResetMotors_Pin SleepMotors_Pin EnableMotors_Pin */
  GPIO_InitStruct.Pin = ResetMotors_Pin|SleepMotors_Pin|EnableMotors_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


// Función de retrollamada (callback) para la interrupción de desbordamiento del temporizador TIM2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // Verificar si la interrupción es del temporizador TIM2
  {
    static uint32_t counter = 0;

    // Mover el motor durante 1 segundo
    if (counter < 1000)
    {
      //    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Iniciar la generación de la señal PWM en el canal 1
      // HAL_GPIO_WritePin(GPIO_Port, DIR_Pin, GPIO_PIN_SET);  // Establecer la dirección del motor (DIR) hacia adelante
      // HAL_GPIO_WritePin(GPIO_Port, STEP_Pin, GPIO_PIN_SET);  // Generar un pulso en el pin de paso (STEP)
      // HAL_GPIO_WritePin(GPIO_Port, STEP_Pin, GPIO_PIN_RESET);  // Restablecer el pulso en el pin de paso (STEP)
    }
    // Detener el motor durante 1 segundo
    else if (counter < 2000)
    {
      //        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Detener la generación de la señal PWM en el canal 1
      // HAL_GPIO_WritePin(GPIO_Port, DIR_Pin, GPIO_PIN_RESET);  // Establecer la dirección del motor (DIR) hacia atrás
      // HAL_GPIO_WritePin(GPIO_Port, STEP_Pin, GPIO_PIN_SET);  // Generar un pulso en el pin de paso (STEP)
      // HAL_GPIO_WritePin(GPIO_Port, STEP_Pin, GPIO_PIN_RESET);  // Restablecer el pulso en el pin de paso (STEP)
    }
    else
    {
      counter = 0;  // Reiniciar el contador para repetir el ciclo
    }

    counter++;
  }

  if (htim->Instance == TIM3)  // Verificar si la interrupción es del temporizador TIM3
  {
	  HAL_GPIO_TogglePin(PinLedPcb_GPIO_Port, PinLedPcb_Pin);		// Cambia el estado del led PC13 cada vez que salta la interrupción
  }
}

// Función de retrollamada (callback) para la interrupción externa EXTI3
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(PinLedPcb_GPIO_Port, PinLedPcb_Pin);
	/*
	 *
	if (GPIO_Pin == GPIO_PIN_9)
  // if (HAL_GPIO_ReadPin(StopM1_GPIO_Port, StopM1_Pin) == GPIO_PIN_RESET)
  {
    // Se detectó un flanco descendente en PA3, detener el motor
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Detener la generación de la señal PWM en el canal 1
    HAL_GPIO_WritePin(EnableMotor_GPIO_Port, EnableMotor_Pin, GPIO_PIN_SET);
  }
  else
  {
    // Se detectó un flanco ascendente en PA3, reanudar el motor
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Iniciar la generación de la señal PWM en el canal 1
    HAL_GPIO_WritePin(EnableMotor_GPIO_Port, EnableMotor_Pin, GPIO_PIN_RESET);
  }

	*/
  HAL_GPIO_EXTI_IRQHandler(StopM1_Pin);  // Limpiar la bandera de interrupción EXTI3
}

/*
// Función para el callback del conversor ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	TIM2->CCR2 = adcVal;
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
