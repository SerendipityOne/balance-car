#include "Motor.h"
#include "gpio.h"

void Motor_Init(void) {
  MX_GPIO_Init();
}

void Motor_SetPWM(int16_t PWM1, int16_t PWM2) {
  if (PWM1 >= 0) {
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN2_Pin, GPIO_PIN_SET);
    PWM_Set_Compare1(PWM1);
  } else {
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN2_Pin, GPIO_PIN_RESET);
    PWM_Set_Compare1(-PWM1);
  }

  if (PWM2 >= 0) {
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN2_Pin, GPIO_PIN_SET);
    PWM_Set_Compare2(PWM2);
  } else {
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN2_Pin, GPIO_PIN_RESET);
    PWM_Set_Compare2(-PWM2);
  }
}
