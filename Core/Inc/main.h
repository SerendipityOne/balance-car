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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder1_PIN1_Pin GPIO_PIN_0
#define Encoder1_PIN1_GPIO_Port GPIOA
#define Encoder1_PIN2_Pin GPIO_PIN_1
#define Encoder1_PIN2_GPIO_Port GPIOA
#define SR04_ECHO_Pin GPIO_PIN_2
#define SR04_ECHO_GPIO_Port GPIOA
#define SR04_ECHO_EXTI_IRQn EXTI2_IRQn
#define SR04_TRIG_Pin GPIO_PIN_3
#define SR04_TRIG_GPIO_Port GPIOA
#define MOTOR1_PIN1_Pin GPIO_PIN_12
#define MOTOR1_PIN1_GPIO_Port GPIOB
#define MOTOR1_PIN2_Pin GPIO_PIN_13
#define MOTOR1_PIN2_GPIO_Port GPIOB
#define MOTOR2_PIN1_Pin GPIO_PIN_14
#define MOTOR2_PIN1_GPIO_Port GPIOB
#define MOTOR2_PIN2_Pin GPIO_PIN_15
#define MOTOR2_PIN2_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOA
#define MPU_SDA_Pin GPIO_PIN_3
#define MPU_SDA_GPIO_Port GPIOB
#define MPU_SDL_Pin GPIO_PIN_4
#define MPU_SDL_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_5
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define Encoder2_PIN1_Pin GPIO_PIN_6
#define Encoder2_PIN1_GPIO_Port GPIOB
#define Encoder2_PIN2_Pin GPIO_PIN_7
#define Encoder2_PIN2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
