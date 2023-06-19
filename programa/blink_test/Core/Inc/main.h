/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LedPcb_Pin GPIO_PIN_13
#define LedPcb_GPIO_Port GPIOC
#define StepM1_Pin GPIO_PIN_0
#define StepM1_GPIO_Port GPIOA
#define StepM2_Pin GPIO_PIN_1
#define StepM2_GPIO_Port GPIOA
#define StepM3_Pin GPIO_PIN_2
#define StepM3_GPIO_Port GPIOA
#define ServoMotor_Pin GPIO_PIN_3
#define ServoMotor_GPIO_Port GPIOA
#define StopM1_Pin GPIO_PIN_4
#define StopM1_GPIO_Port GPIOA
#define StopM2_Pin GPIO_PIN_5
#define StopM2_GPIO_Port GPIOA
#define StopM3_Pin GPIO_PIN_6
#define StopM3_GPIO_Port GPIOA
#define ADC1_IN7_Pin GPIO_PIN_7
#define ADC1_IN7_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
