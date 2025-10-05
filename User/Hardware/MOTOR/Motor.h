#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "PWM.h"

#define MOTOR_PROT GPIOB
#define MOTOR1_PIN1 GPIO_PIN_13
#define MOTOR1_PIN2 GPIO_PIN_12
#define MOTOR2_PIN1 GPIO_PIN_14
#define MOTOR2_PIN2 GPIO_PIN_15

void Motor_Init(void);
void Motor_SetPWM(int16_t PWM);

#endif  // !__MOTOR_H
