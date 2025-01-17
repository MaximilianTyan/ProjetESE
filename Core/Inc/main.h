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
#include "stm32l0xx_hal.h"

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
#define IR_SENSOR_IN_Pin GPIO_PIN_0
#define IR_SENSOR_IN_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define DIR_CTRL_Pin GPIO_PIN_12
#define DIR_CTRL_GPIO_Port GPIOB
#define STEP_PWM_Pin GPIO_PIN_13
#define STEP_PWM_GPIO_Port GPIOB
#define NFC_RST_Pin GPIO_PIN_9
#define NFC_RST_GPIO_Port GPIOA
#define LED1_CTRL_Pin GPIO_PIN_3
#define LED1_CTRL_GPIO_Port GPIOB
#define LED2_CTRL_Pin GPIO_PIN_4
#define LED2_CTRL_GPIO_Port GPIOB
#define LED3_CTRL_Pin GPIO_PIN_5
#define LED3_CTRL_GPIO_Port GPIOB
#define LED4_CTRL_Pin GPIO_PIN_6
#define LED4_CTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
