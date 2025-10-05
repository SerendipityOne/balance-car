#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "main.h"

void HC_SR04_Init(void);
float HC_SR04_GetDistance(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif  // !__HC_SR04_H
