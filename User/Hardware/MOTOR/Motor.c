#include "Motor.h"
#include "gpio.h"

void Motor_Init(void) {
  MX_GPIO_Init();
}

/**
 * @brief 设置电机PWM值，控制电机转动方向和速度
 * @param PWM PWM值，正数表示正转，负数表示反转，绝对值决定速度大小
 */
void Motor_SetPWM(int16_t PWM) {
  if (PWM > 0) {                                                                // 当PWM值为正时，电机正转
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN1_Pin, GPIO_PIN_RESET);  // 设置电机1的引脚1为低电平
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN2_Pin, GPIO_PIN_SET);    // 设置电机1的引脚2为高电平
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN1_Pin, GPIO_PIN_RESET);  // 设置电机2的引脚1为低电平
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN2_Pin, GPIO_PIN_SET);    // 设置电机2的引脚2为高电平
    PWM_Set_Compare(PWM);                                                       // 设置PWM比较值为正，控制电机正转速度
  } else {                                                                      // 当PWM值为负时，电机反转
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN1_Pin, GPIO_PIN_SET);    // 设置电机1的引脚1为高电平
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN2_Pin, GPIO_PIN_RESET);  // 设置电机1的引脚2为低电平
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN1_Pin, GPIO_PIN_SET);    // 设置电机2的引脚1为高电平
    HAL_GPIO_WritePin(MOTOR2_PIN1_GPIO_Port, MOTOR2_PIN2_Pin, GPIO_PIN_RESET);  // 设置电机2的引脚2为低电平
    PWM_Set_Compare(-PWM);                                                      // 设置PWM比较值为正（取绝对值），控制电机反转速度
  }
}
