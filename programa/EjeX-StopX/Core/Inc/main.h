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
#define StepM_X_Pin GPIO_PIN_0
#define StepM_X_GPIO_Port GPIOA
#define azul_Pin GPIO_PIN_2
#define azul_GPIO_Port GPIOA
#define DirM_X_Pin GPIO_PIN_3
#define DirM_X_GPIO_Port GPIOA
#define StopM_X_Pin GPIO_PIN_14
#define StopM_X_GPIO_Port GPIOB
#define StopM_X_EXTI_IRQn EXTI15_10_IRQn
#define ResetMotors_Pin GPIO_PIN_15
#define ResetMotors_GPIO_Port GPIOB
#define SleepMotors_Pin GPIO_PIN_5
#define SleepMotors_GPIO_Port GPIOB
#define EnableMotors_Pin GPIO_PIN_8
#define EnableMotors_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
