#include "HC_SR04.h"
#include "Delay.h"

extern TIM_HandleTypeDef htim3;

uint32_t echo_start = 0;  // 上升沿时刻的TIM3计数值
uint32_t echo_end = 0;    // 下降沿时刻的TIM3计数值
uint8_t echo_state = 0;   // 0:等待上升沿, 1:等待下降沿
float temp_distance = 0;  // 测量的距离值

/**
  * @brief  HC-SR04超声波传感器初始化函数
  * @param  无
  * @retval 无
  * @note   启动TIM3定时器，用于测量超声波回波脉冲宽度
  */
void HC_SR04_Init(void) {
  HAL_TIM_Base_Start(&htim3);
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
 * @brief HC_SR04超声波传感器中断处理函数，在GPIO_EXIT中断中调用
 * @param 无
 * @retval 无
 */
void HC_SR04_IRQHandler(void) {
  if (echo_state == 0) {                         // 等待上升沿
    __HAL_TIM_SET_COUNTER(&htim3, 0);            // 清零TIM3计数值
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
