#include "Encoder.h"

/**
  * @brief  初始化编码器1和编码器2
  * @param  无
  * @retval 无
  */
void Encoder_Init(void) {
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}


/**
  * @brief  获取编码器1计数值并清零
  * @retval 编码器计数值增量
  */
int16_t Encoder1_Get(void) {
  int16_t temp;
  temp = __HAL_TIM_GET_COUNTER(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  return temp;
}


/**
  * @brief  获取编码器2计数值并清零
  * @retval 编码器计数值增量
  */
int16_t Encoder2_Get(void) {
  int16_t temp;
  temp = __HAL_TIM_GET_COUNTER(&htim4);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  return -temp;
}
