/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "Encoder.h"
#include "HC_SR04.h"
#include "MPU6050.h"
#include "Motor.h"
#include "OLED.h"
#include "PID.h"
#include "PWM.h"
#include "Serial_DMA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MACHINE_MEDIAN -1.0f  // 机械中值
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t run_status = 1;  // 0:停止,1:运行PID
MPU mpu_data;
int16_t speed1, speed2, motor1_pwm, motor2_pwm;
float angle;  // 直立角度，用于直立环控制
PID_t vertical_pid = {
    .Kp = 250,  // 420
    .Ki = 0,
    .Kd = 0.46,  //0.2
    .Target = MACHINE_MEDIAN,
};  // 垂直方向速度控制PID

PID_t speed_pid = {
    .Kp = 0.66,
    .Ki = 0.0033,
    .Kd = 0,
    .Target = 0,
};  // 速度控制PID
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void All_Init(void) {
  OLED_Init();
  MPU_DMP_Init();
  PWM_Init();
  Encoder_Init();
  HC_SR04_Init();
  Serial_DMA_Init();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  All_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    OLED_Printf(0, 0, OLED_8X16, "roll:%.2f", mpu_data.roll);
    OLED_Printf(0, 16, OLED_8X16, "out:%+04d", motor1_pwm);
    OLED_Printf(0, 32, OLED_8X16, "gyrox:%+05d", mpu_data.gyrox);
    OLED_Printf(0, 48, OLED_8X16, "vt:%.2f", vertical_pid.Target);

    // OLED_Printf(0, 0, OLED_6X8, "pitch:%.2f", mpu_data.pitch);
    // OLED_Printf(0, 8, OLED_6X8, "roll:%.2f", mpu_data.roll);
    // OLED_Printf(0, 16, OLED_6X8, "yaw:%.2f", mpu_data.yaw);
    // OLED_Printf(0, 24, OLED_6X8, "gx:%+05d", mpu_data.gyrox);
    // OLED_Printf(0, 32, OLED_6X8, "gy:%+05d", mpu_data.gyroy);
    // OLED_Printf(0, 40, OLED_6X8, "gz:%+05d", mpu_data.gyroz);
    // OLED_Printf(0, 48, OLED_6X8, "ax:%+05d", mpu_data.accelx);
    // OLED_Printf(0, 56, OLED_6X8, "ay:%+05d", mpu_data.accely);
    // OLED_Printf(0, 64, OLED_6X8, "az:%+05d", mpu_data.accelz);

    // Serial_DMA_Printf("%f,%f", mpu_data.roll, vertical_pid.Target);
    OLED_Update();
    delay_ms(100);

    // Serial_DMA_Printf("%.2f,%.2f\r\n", mpu_data.roll, vertical_pid.Out);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_INT_Pin) {
    if (run_status) {
      /* 内环直立环PID（10ms执行一次）*/
      MPU_DMP_ReadData(&mpu_data);
      angle = mpu_data.roll;

      /* 外环速度环*/
      speed1 = Encoder1_Get();
      speed2 = Encoder2_Get();
      speed_pid.Actual = speed1 + speed2;
      Speed_PID_Update(&speed_pid);
      vertical_pid.Target = MACHINE_MEDIAN - speed_pid.Out;

      /* 直立环 */
      vertical_pid.Actual = angle;
      Vertical_PID_Update(&vertical_pid, mpu_data.gyrox);
      motor1_pwm = vertical_pid.Out;
      motor2_pwm = vertical_pid.Out;
      Motor_SetPWM(motor1_pwm, motor2_pwm);

      // 倒下停止
      if (fabs(mpu_data.roll) > 45) {
        run_status = 0;
      }

    } else {
      Motor_SetPWM(0, 0);
    }
  }
}

/* 10ms一次 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
//   if (htim->Instance == TIM3) {
//     }
// }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
