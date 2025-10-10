#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

void Encoder_Init(void);
int16_t Encoder_Get(uint8_t id);

#endif // !__ENCODER_H
