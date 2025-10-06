#include "PID.h"

/**
 * PID控制器更新函数
 * 根据目标值和实际值计算PID控制输出
 * @param p PID控制器结构体指针
 */
void PID_Update(PID_t* p) {
  // 保存上一次的误差，用于计算微分项
  p->Error1 = p->Error0;
  // 计算当前误差
  p->Error0 = p->Target - p->Actual;

  // 积分项处理：如果Ki不为0，则进行积分累积；否则清零积分项
  if (p->Ki != 0) {
    p->ErrorInt += p->Error0;
  } else {
    p->ErrorInt = 0;
  }

  // 计算PID输出：比例项 + 积分项 + 微分项
  p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

  // 输出限幅处理：防止输出超出设定范围
  if (p->Out > p->OutMax) p->Out = p->OutMax;
  if (p->Out < p->OutMin) p->Out = p->OutMin;
}
