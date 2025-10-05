#include "Motor.h"
#include "gpio.h"

void Motor_Init(void) {
  MX_GPIO_Init();
}

void Motor_SetPWM(int16_t PWM) {
  if (PWM > 0) {
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR1_PIN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR1_PIN2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR2_PIN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR2_PIN2, GPIO_PIN_SET);
    PWM_Set_Compare(PWM);
  } else {
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR1_PIN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR1_PIN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR2_PIN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PROT, MOTOR2_PIN2, GPIO_PIN_RESET);
    PWM_Set_Compare(-PWM);
  }
}
