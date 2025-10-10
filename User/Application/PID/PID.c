#include "PID.h"

/**
 * @brief  更新直立环PID控制器输出
 * @param  p: PID控制器结构体指针
 * @param  gyro_x: 陀螺仪X轴角速度数据，用于微分项计算
 * @retval 无
 * 
 * 该函数实现直立环PID控制，使用陀螺仪数据作为微分反馈，
 * 通过PD控制算法计算输出值，用于保持平衡车直立状态
 */
void Vertical_PID_Update(PID_t* p, short gyro_x) {
  p->Error0 = p->Target - p->Actual;
  p->Out = p->Kp * p->Error0 - p->Kd * gyro_x;
}

/**
 * @brief  更新速度环PID控制器输出
 * @param  p: PID控制器结构体指针
 * @retval 无
 * 
 * 该函数实现速度环PID控制，包含误差计算、低通滤波、积分限幅等处理，
 * 通过PI控制算法计算输出值，用于控制平衡车的速度
 */
void Speed_PID_Update(PID_t* p) {
  const float a = 0.7f;  // 低通滤波系数

  // 计算误差
  p->Error0 = p->Target - p->Actual;

  // 滤波器
  p->Error0 = a * p->Error0 + (1 - a) * p->Error1;
  p->Error1 = p->Error0;

  p->ErrorInt += p->Error0;

  // 积分限幅
  if (p->ErrorInt > 10000) p->ErrorInt = 10000;
  if (p->ErrorInt < -10000) p->ErrorInt = -10000;

  p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt;
}
