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
  * @brief  获取指定编码器的计数值并清零
  * @param  id: 编码器编号，1表示第一个编码器，2表示第二个编码器
  * @retval 编码器计数值增量，对于第二个编码器会返回负值
  */
int16_t Encoder_Get(uint8_t id) {
  int16_t temp;

  if (id == 1) {  // 1号电机
    temp = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

  } else if (id == 2) {  // 2号电机
    temp = -__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
  }
  return temp;
}
