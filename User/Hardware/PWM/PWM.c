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
 * @brief 设置定时器1通道1的PWM比较值
 * @param compare 比较值，用于控制PWM输出的占空比
 * @note 此函数通过HAL库函数设置定时器1通道1的比较值，从而调整PWM输出的占空比
 */
void PWM_Set_Compare1(uint16_t compare) {  // 设置PWM比较值的函数，参数compare为比较值
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}

/**
 * @brief 设置定时器1通道4的PWM比较值
 * @param compare 比较值，用于控制PWM输出的占空比
 * @note 此函数通过HAL库函数设置定时器1通道4的比较值，从而调整PWM输出的占空比
 */
void PWM_Set_Compare2(uint16_t compare) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, compare);
}
