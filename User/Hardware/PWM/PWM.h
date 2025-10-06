#ifndef __PWM_H
#define __PWM_H

#include "main.h"
extern TIM_HandleTypeDef htim1;

void PWM_Init(void);
void PWM_Set_Compare1(uint16_t compare);
void PWM_Set_Compare2(uint16_t compare);

#endif
