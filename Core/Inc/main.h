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
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


#define FREQ 72000000

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
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define PC14_Pin GPIO_PIN_14
#define PC14_GPIO_Port GPIOC
#define PC15_Pin GPIO_PIN_15
#define PC15_GPIO_Port GPIOC
#define En2_Pin GPIO_PIN_0
#define En2_GPIO_Port GPIOA
#define Button_Pin GPIO_PIN_4
#define Button_GPIO_Port GPIOA
#define En1_Pin GPIO_PIN_5
#define En1_GPIO_Port GPIOA
#define Dir1_Pin GPIO_PIN_6
#define Dir1_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_0
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_1
#define CS2_GPIO_Port GPIOB
#define Dir2_Pin GPIO_PIN_10
#define Dir2_GPIO_Port GPIOB
#define Led1_Pin GPIO_PIN_11
#define Led1_GPIO_Port GPIOB
#define EndCap1_Pin GPIO_PIN_12
#define EndCap1_GPIO_Port GPIOB
#define EndCap2_Pin GPIO_PIN_14
#define EndCap2_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_15
#define S1_GPIO_Port GPIOB
#define EndCap4_Pin GPIO_PIN_11
#define EndCap4_GPIO_Port GPIOA
#define Buser_Pin GPIO_PIN_12
#define Buser_GPIO_Port GPIOA
#define Reserved_Pin GPIO_PIN_15
#define Reserved_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_6
#define S2_GPIO_Port GPIOB
#define EndCap3_Pin GPIO_PIN_7
#define EndCap3_GPIO_Port GPIOB
#define En3_Pin GPIO_PIN_8
#define En3_GPIO_Port GPIOB
#define Dir3_Pin GPIO_PIN_9
#define Dir3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
