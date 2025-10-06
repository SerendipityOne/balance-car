#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "PWM.h"

void Motor_Init(void);
void Motor_SetPWM(int16_t PWM1, int16_t PWM2);

#endif  // !__MOTOR_H
