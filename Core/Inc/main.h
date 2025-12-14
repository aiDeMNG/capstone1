/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define MQ135_Pin GPIO_PIN_0
#define MQ135_GPIO_Port GPIOA
#define motor1_Pin GPIO_PIN_1
#define motor1_GPIO_Port GPIOA
#define motor1A2_Pin GPIO_PIN_2
#define motor1A2_GPIO_Port GPIOA
#define motor1A3_Pin GPIO_PIN_3
#define motor1A3_GPIO_Port GPIOA
#define motor1A4_Pin GPIO_PIN_4
#define motor1A4_GPIO_Port GPIOA
#define motor1A5_Pin GPIO_PIN_5
#define motor1A5_GPIO_Port GPIOA
#define fans_Pin GPIO_PIN_6
#define fans_GPIO_Port GPIOA
#define DHT22_Pin GPIO_PIN_12
#define DHT22_GPIO_Port GPIOB
#define motor2_Pin GPIO_PIN_8
#define motor2_GPIO_Port GPIOA
#define motor2A9_Pin GPIO_PIN_9
#define motor2A9_GPIO_Port GPIOA
#define motor2A10_Pin GPIO_PIN_10
#define motor2A10_GPIO_Port GPIOA
#define motor2A11_Pin GPIO_PIN_11
#define motor2A11_GPIO_Port GPIOA
#define bluetooth_Pin GPIO_PIN_6
#define bluetooth_GPIO_Port GPIOB
#define bluetoothB7_Pin GPIO_PIN_7
#define bluetoothB7_GPIO_Port GPIOB
#define GY_30_Pin GPIO_PIN_8
#define GY_30_GPIO_Port GPIOB
#define GY_30B9_Pin GPIO_PIN_9
#define GY_30B9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
