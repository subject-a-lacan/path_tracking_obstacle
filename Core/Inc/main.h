/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define PERIODIC(T) \
    static uint32_t nxt = 0; \
    if(HAL_GetTick() < nxt) return; \
    nxt += (T);
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Trigger_Pin GPIO_PIN_0
#define Trigger_GPIO_Port GPIOA
#define Echo_Pin GPIO_PIN_1
#define Echo_GPIO_Port GPIOA
#define LORA_TX_Pin GPIO_PIN_2
#define LORA_TX_GPIO_Port GPIOA
#define LORA_RX_Pin GPIO_PIN_3
#define LORA_RX_GPIO_Port GPIOA
#define BUZZLER_Pin GPIO_PIN_4
#define BUZZLER_GPIO_Port GPIOA
#define AD2_Pin GPIO_PIN_6
#define AD2_GPIO_Port GPIOA
#define AD1_Pin GPIO_PIN_7
#define AD1_GPIO_Port GPIOA
#define AD0_Pin GPIO_PIN_0
#define AD0_GPIO_Port GPIOB
#define ganwei_enable_Pin GPIO_PIN_1
#define ganwei_enable_GPIO_Port GPIOB
#define KEY_1_Pin GPIO_PIN_12
#define KEY_1_GPIO_Port GPIOB
#define Lora_AUX_Pin GPIO_PIN_15
#define Lora_AUX_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_8
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_9
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_10
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOA
#define KEY_2_Pin GPIO_PIN_12
#define KEY_2_GPIO_Port GPIOA
#define steer_21_Pin GPIO_PIN_15
#define steer_21_GPIO_Port GPIOA
#define steer_22_Pin GPIO_PIN_3
#define steer_22_GPIO_Port GPIOB
#define A_31_Pin GPIO_PIN_4
#define A_31_GPIO_Port GPIOB
#define B_32_Pin GPIO_PIN_5
#define B_32_GPIO_Port GPIOB
#define A_41_Pin GPIO_PIN_6
#define A_41_GPIO_Port GPIOB
#define A_42_Pin GPIO_PIN_7
#define A_42_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern volatile uint16_t flag_avoid_reset;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
