#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "main.h"

void HC_SR04_Init(void);
float HC_SR04_GetDistance(void);
void HC_SR04_IRQHandler(void);

#endif  // !__HC_SR04_H
