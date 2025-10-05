#ifndef __PWM_H
#define __PWM_H

#include "main.h"
extern TIM_HandleTypeDef htim1;

void PWM_Init(void);
void PWM_Set_Compare(uint16_t compare);

#endif
