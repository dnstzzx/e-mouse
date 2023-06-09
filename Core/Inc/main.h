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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOE
#define BTN_WKUP_Pin GPIO_PIN_0
#define BTN_WKUP_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOA
#define MPU6050_INT_Pin GPIO_PIN_5
#define MPU6050_INT_GPIO_Port GPIOC
#define MPU6050_INT_EXTI_IRQn EXTI9_5_IRQn
#define HMC5883L_INT_Pin GPIO_PIN_1
#define HMC5883L_INT_GPIO_Port GPIOB
#define HMC5883L_INT_EXTI_IRQn EXTI1_IRQn
#define VL53_7_SHUT_Pin GPIO_PIN_14
#define VL53_7_SHUT_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOG
#define VL53_6_SHUT_Pin GPIO_PIN_6
#define VL53_6_SHUT_GPIO_Port GPIOG
#define MOTOR_IN_L2_Pin GPIO_PIN_6
#define MOTOR_IN_L2_GPIO_Port GPIOC
#define MOTOR_IN_L1_Pin GPIO_PIN_7
#define MOTOR_IN_L1_GPIO_Port GPIOC
#define MOTOR_IN_R2_Pin GPIO_PIN_8
#define MOTOR_IN_R2_GPIO_Port GPIOC
#define MOTOR_IN_R1_Pin GPIO_PIN_9
#define MOTOR_IN_R1_GPIO_Port GPIOC
#define VL53_3_SHUT_Pin GPIO_PIN_15
#define VL53_3_SHUT_GPIO_Port GPIOA
#define VL53_2_SHUT_Pin GPIO_PIN_11
#define VL53_2_SHUT_GPIO_Port GPIOC
#define VL53_5_SHUT_Pin GPIO_PIN_1
#define VL53_5_SHUT_GPIO_Port GPIOD
#define VL53_1_SHUT_Pin GPIO_PIN_2
#define VL53_1_SHUT_GPIO_Port GPIOD
#define VL53_4_SHUT_Pin GPIO_PIN_5
#define VL53_4_SHUT_GPIO_Port GPIOD
#define VL53_8_SHUT_Pin GPIO_PIN_6
#define VL53_8_SHUT_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOD
#define MOTOR_ENC_L1_Pin GPIO_PIN_4
#define MOTOR_ENC_L1_GPIO_Port GPIOB
#define MOTOR_ENC_L2_Pin GPIO_PIN_5
#define MOTOR_ENC_L2_GPIO_Port GPIOB
#define MOTOR_ENC_R1_Pin GPIO_PIN_6
#define MOTOR_ENC_R1_GPIO_Port GPIOB
#define MOTOR_ENC_R2_Pin GPIO_PIN_7
#define MOTOR_ENC_R2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
