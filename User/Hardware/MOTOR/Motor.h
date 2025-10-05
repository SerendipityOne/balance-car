#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "PWM.h"

void Motor_Init(void);
void Motor_SetPWM(int16_t PWM);

#endif  // !__MOTOR_H
