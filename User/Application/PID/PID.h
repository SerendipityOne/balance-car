#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct PID_t {
  float Target, Actual, Out;
  float Kp, Ki, Kd;
  float Error0, Error1, ErrorInt;

  float OutMax, OutMin;
} PID_t;

void Vertical_PID_Update(PID_t* p, short gyro_y);
void Speed_PID_Update(PID_t* p);

#endif  // !__PID_H
