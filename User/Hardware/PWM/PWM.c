#include "PWM.h"

/**
 * @brief PWM初始化函数
 * @note  该函数用于启动定时器1的PWM输出，并开启中断
 *        具体通道为通道1和通道4
 */
void PWM_Init(void) {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 启动定时器1的PWM输出，使用通道1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 启动定时器1的PWM输出，使用通道4
}

/**
 * @brief  设置PWM比较值，控制PWM输出占空比
 * @note   该函数用于设置定时器1通道1和通道4的比较值，从而控制PWM输出占空比
 * @param  compare: 要设置的比较值，决定PWM波形的占空比
 * @retval 无
 */
void PWM_Set_Compare(uint16_t compare) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, compare);
}
