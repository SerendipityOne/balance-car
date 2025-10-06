#include "HC_SR04.h"
#include "Delay.h"

extern TIM_HandleTypeDef htim3;

uint32_t echo_start = 0;  // 上升沿时刻的TIM3计数值
uint32_t echo_end = 0;    // 下降沿时刻的TIM3计数值
uint8_t echo_state = 0;   // 0:等待上升沿, 1:等待下降沿
float temp_distance = 0;       // 测量的距离值

/**
  * @brief  HC-SR04超声波传感器初始化函数
  * @param  无
  * @retval 无
  * @note   启动TIM3定时器，用于测量超声波回波脉冲宽度
  */
void HC_SR04_Init(void) {
  HAL_TIM_Base_Start_IT(&htim3);
}

/**
  * @brief  获取HC-SR04超声波传感器测量的距离
  * @param  无
  * @retval float 测量的距离值，单位为厘米(cm)
  * @note   根据脉冲宽度计算距离，声波速度为340m/s，即0.034cm/us
  */
float HC_SR04_GetDistance(void) {
  // 发送触发信号
  HAL_GPIO_WritePin(SR04_TRIG_GPIO_Port, SR04_TRIG_Pin, GPIO_PIN_SET);
  Delay_us(10);  // 至少10us
  HAL_GPIO_WritePin(SR04_TRIG_GPIO_Port, SR04_TRIG_Pin, GPIO_PIN_RESET);

  return temp_distance;
}

/**
  * @brief  外部中断回调函数，用于测量HC-SR04的回波脉冲宽度
  * @param  GPIO_Pin 触发中断的GPIO引脚
  * @retval 无
  * @note   通过检测ECHO引脚的上升沿和下降沿来测量脉冲宽度
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == SR04_ECHO_Pin) {
    if (echo_state == 0) {                         // 等待上升沿
      echo_start = __HAL_TIM_GET_COUNTER(&htim3);  // 记录上升沿时刻的计数值
      echo_state = 1;                              // 进入等待下降沿状态
    } else if (echo_state == 1) {                  // 等待下降沿
      // 记录下降沿时刻的计数值
      echo_end = __HAL_TIM_GET_COUNTER(&htim3);

      // 距离 = 脉冲宽度 * 声速 / 2
      // 声速为340m/s = 0.034cm/us
      // 除以2是因为超声波需要往返一次
      temp_distance = (float)(echo_end - echo_start) * 0.034f / 2.0f;

      echo_state = 0;  // 恢复到等待上升沿状态
    }
  }
}
