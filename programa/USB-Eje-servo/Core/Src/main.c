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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

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

//	Vel Motors
uint16_t velMotor_X = 0;
uint16_t velMotor_Y = 0;
uint16_t velMotor_Z = 0;

//	Dir Motors
uint8_t DirM_X = 0;
uint8_t DirM_Y = 0;
uint8_t DirM_Z = 0;

// Step Motors
float thetaMotor_X = 1;
float thetaMotor_Y = 1;
float thetaMotor_Z = 1;

//	FLAGS STOP GLOBALES
uint8_t flagStopM_X = 0;
uint8_t flagStopM_Y = 0;
uint8_t flagStopM_Z = 0;

//	FLAGS OPTICAL LIMIT SWITCH SENSOR PER AXIS GLOBALES
uint8_t opticalLimitSwitchSensor_X = 0;
uint8_t opticalLimitSwitchSensor_Y = 0;
uint8_t opticalLimitSwitchSensor_Z = 0;

///// Factor de microStepping
uint8_t microSteppingM_X = 1;
uint8_t microSteppingM_Y = 1;
uint8_t microSteppingM_Z = 1;

// uint32_t adcVal;					// Arreglo para guardar los valores leidos del conversor ADC
uint32_t contador = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int HomingMotors(uint8_t* hmX, uint8_t* hmY, uint8_t* hmZ);
void ActivatedAll (void);
float deg2rad(float degrees);

//void moverMotor_eje(char eje, float *thetaTarget, uint16_t velocidadTarget);
void moverX(float target, uint16_t velocidad);

// extras
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);		// Función para el manejo de interrupciones del Timer 2
// void EXTI9_5_IRQHandler(void);		// Función para manejo de la interrupciones externas por fin de carrera
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  ///// Configurar los Buses Reset, Sleep y Enable
  HAL_GPIO_WritePin(ResetMotors_GPIO_Port, ResetMotors_Pin, SET);
  HAL_GPIO_WritePin(SleepMotors_GPIO_Port, SleepMotors_Pin, SET);
  HAL_GPIO_WritePin(EnableMotors_GPIO_Port, EnableMotors_Pin, SET);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);			// Iniciar el temporizador con interrupción

/*
  TIM2->ARR = 8000;
  TIM2->CCR1 = 7500;

  TIM4->ARR = 9999;
  TIM4->CCR4 = 5000;
*/

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim4);			// Iniciar el temporizador con interrupción

  ///// Flags de Homing
  uint8_t homeMotor_X = 0;
  uint8_t homeMotor_Y = 0;
  uint8_t homeMotor_Z = 0;
  int countHome = 0;

  ///// Thetas objetivos
  float thetaTargetMotor_X = 0;
  float thetaTargetMotor_Y = 0;
  float thetaTargetMotor_Z = 0;

  uint16_t velocidadTargetMotor_X = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Caso de uso sin Motor Y
	  flagStopM_Y = 1;
	  thetaTargetMotor_Y = 0;
	  // Caso de uso sin Motor Z
	  flagStopM_Z = 1;
	  thetaTargetMotor_Z = 0;

	  //HAL_Delay(1000);
	  if (countHome == 0){
		  countHome = HomingMotors(&homeMotor_X, &homeMotor_Y, &homeMotor_Z);
		  HAL_Delay(700);
		  HAL_GPIO_WritePin(azul_GPIO_Port, azul_Pin, RESET);
	  }

	  // Movimiento consigna
	  thetaTargetMotor_X = 25.0;
	  velocidadTargetMotor_X = 5;
	  moverX(thetaTargetMotor_X, velocidadTargetMotor_X);
	  HAL_Delay(1000);
	  thetaTargetMotor_X = 50.0;
	  velocidadTargetMotor_X = 25;
	  moverX(thetaTargetMotor_X, velocidadTargetMotor_X);
	  HAL_Delay(1000);
	  thetaTargetMotor_X = 0.0;
	  velocidadTargetMotor_X = 20;
	  moverX(thetaTargetMotor_X, velocidadTargetMotor_X);
	  HAL_Delay(1000);
	  /*
	  thetaTargetMotor_X = 100.0;
	  velocidadTargetMotor_X = 50;
	  moverX(thetaTargetMotor_X, velocidadTargetMotor_X);
	  HAL_Delay(1000);

	  /*
	  int miCcr = 0;  // Inicializamos el valor del CCR1 en 0
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Iniciamos la generación de PWM en el canal 1
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	  for (int var = 0; var <= 100; var = var + 10)
	  {
	    miCcr = (10000 * var) / 100;
	    TIM4->CCR4 = miCcr;
	    TIM2->CCR1 = miCcr;
	    HAL_Delay(1000);
	  }
	    TIM4->CCR4 = 0;
	    TIM2->CCR1 = 0;
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Detenemos la generación de PWM en el canal 1
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	  HAL_GPIO_TogglePin(DirM_X_GPIO_Port, DirM_X_Pin);
	  HAL_Delay(1000);
	  */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
