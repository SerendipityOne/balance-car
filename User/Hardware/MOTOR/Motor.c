#include "Motor.h"
#include "gpio.h"

void Motor_Init(void) {
  MX_GPIO_Init();
}

/**
  * @brief  设置两个电机的PWM输出值
  * @param  PWM1: 电机1的PWM值，正值表示正转，负值表示反转，绝对值决定速度
  * @param  PWM2: 电机2的PWM值，正值表示正转，负值表示反转，绝对值决定速度
  * @retval 无
  * @note   通过控制H桥驱动电路的GPIO引脚和PWM比较值来控制电机的转向和转速
  */
void Motor_SetPWM(int16_t PWM1, int16_t PWM2) {
  /* 控制电机1转向和转速 */
  if (PWM1 >= 0) {
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN2_Pin, GPIO_PIN_RESET);

    PWM_Set_Compare1(PWM1);
  } else {
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR1_PIN1_GPIO_Port, MOTOR1_PIN2_Pin, GPIO_PIN_SET);
    PWM_Set_Compare1(-PWM1);
  }

  /* 控制电机2转向和转速 */
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
