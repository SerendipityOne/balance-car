#include "Delay.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus) {
  __IO uint32_t Delay = xus * 72 / 8;  //(SystemCoreClock / 8U / 1000000U)
  //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do {
    __NOP();
  } while (Delay--);
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms) {
  while (xms--) {
    Delay_us(1000);
  }
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs) {
  while (xs--) {
    Delay_ms(1000);
  }
}
