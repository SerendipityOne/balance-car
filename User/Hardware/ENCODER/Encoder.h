#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

void Encoder_Init(void);
int16_t Encoder1_Get(void);
int16_t Encoder2_Get(void);

#endif // !__ENCODER_H
