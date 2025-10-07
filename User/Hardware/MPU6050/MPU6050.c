#include "MPU6050.h"
#include <math.h>
#include "inv_mpu_dmp_motion_driver.h"

#define q30 1073741824.0f

/**
 * @brief 初始化MPU6050传感器
 * @return 0表示初始化成功，1表示初始化失败
 */
uint8_t MPU_Init(void) {
  uint8_t res;  // 用于存储读取的设备ID

  MPU_IIC_Init();                           //初始化IIC总线
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);  //复位MPU6050
  delay_ms(100);                            //等待复位完成
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);  //唤醒MPU6050
  MPU_Set_Gyro_Fsr(3);                      //陀螺仪传感器,±2000dps
  MPU_Set_Accel_Fsr(0);                     //加速度传感器,±2g
  MPU_Set_Rate(200);                        //设置采样率50Hz
  MPU_Write_Byte(MPU_INT_EN_REG, 0X00);     //关闭所有中断
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);  //I2C主模式关闭
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);    //关闭FIFO
  MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);  //INT引脚低电平有效
  res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
  if (res == MPU_ADDR)  //器件ID正确
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);  //设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);  //加速度与陀螺仪都工作
    MPU_Set_Rate(100);                        //设置采样率为100Hz
  } else
    return 1;
  return 0;
}

/**
 * @brief MPU_DMP初始化函数
 * @return uint8_t 初始化状态码，0表示成功，非0表示失败
 */
uint8_t MPU_DMP_Init(void) {
  uint8_t res;  // 用于存储初始化结果的状态变量

  // 初始化MPU传感器
  res = MPU_Init();
  if (res != 0) {
    return res;  // 如果MPU初始化失败，直接返回错误码
  }
  // 初始化MPU的DMP（数字运动处理器）功能
  res = mpu_dmp_init();
  if (res != 0) {
    return res;  // 如果DMP初始化失败，直接返回错误码
  }
  return 0;  // 所有初始化成功，返回0表示成功
}

/**
 * @brief 获取MPU设备的ID
 * @return uint8_t 返回MPU设备的ID值
 */
uint8_t MPU_GetID(void) {
  return MPU_Read_Byte(MPU_DEVICE_ID_REG);  // 读取MPU设备ID寄存器的值并返回
}

/**
 * @brief 设置陀螺仪满量程范围
 * @param fsr 陀螺仪满量程范围设置值 fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * @return uint8_t 操作状态，0表示成功，非0表示失败
 */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr) {
  return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);  //设置陀螺仪满量程范围
}

/**
 * 设置MPU6050加速度传感器满量程范围
 * @param fsr 满量程范围参数(0-3对应±2g/±4g/±8g/±16g)
 * @return 0-设置成功, 其他-设置失败
 */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr) {
  return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);  //设置加速度传感器满量程范围
}

/**
 * @brief 设置MPU6050的数字低通滤波器
 * @param lpf 数字低通滤波频率(Hz)
 * @return 0表示设置成功，其他值表示设置失败
 */
uint8_t MPU_Set_LPF(uint16_t lpf) {
  uint8_t data = 0;
  if (lpf >= 188)
    data = 1;
  else if (lpf >= 98)
    data = 2;
  else if (lpf >= 42)
    data = 3;
  else if (lpf >= 20)
    data = 4;
  else if (lpf >= 10)
    data = 5;
  else
    data = 6;
  return MPU_Write_Byte(MPU_CFG_REG, data);  //设置数字低通滤波器
}

/**
 * @brief 设置MPU6050的采样率(假定Fs=1KHz)
 * @param rate 采样率，范围为4~1000(Hz)
 * @return 0表示设置成功，其他值表示设置失败
 */
uint8_t MPU_Set_Rate(uint16_t rate) {
  uint8_t data;
  if (rate > 1000) rate = 1000;
  if (rate < 4) rate = 4;
  data = 1000 / rate - 1;
  data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);  //设置数字低通滤波器
  return MPU_Set_LPF(rate / 2);                      //自动设置LPF为采样率的一半
}

/**
 * @brief 得到温度值
 * @return 温度值(扩大了100倍)
 */
short MPU_Get_Temperature(void) {
  uint8_t buf[2];
  short raw;
  float temp;
  MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
  raw = ((uint16_t)buf[0] << 8) | buf[1];
  temp = 36.53 + ((double)raw) / 340;
  return temp * 100;
  ;
}

/**
 * @brief 得到陀螺仪值(原始值)
 * @param mpu 指向MPU结构体的指针，用于存储陀螺仪x,y,z轴的原始读数
 * @return 0表示读取成功，其他值为错误代码
 */
uint8_t MPU_Get_Gyroscope(MPU* mpu) {
  uint8_t buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
  if (res == 0) {
    mpu->gyrox = ((uint16_t)buf[0] << 8) | buf[1];
    mpu->gyroy = ((uint16_t)buf[2] << 8) | buf[3];
    mpu->gyroz = ((uint16_t)buf[4] << 8) | buf[5];
  }
  return res;
  ;
}

/**
 * @brief 得到加速度值(原始值)
 * @param mpu 指向MPU结构体的指针，用于存储加速度x,y,z轴的原始读数
 * @return 0表示读取成功，其他值为错误代码
 */
uint8_t MPU_Get_Accelerometer(MPU* mpu) {
  uint8_t buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
  if (res == 0) {
    mpu->accelx = ((uint16_t)buf[0] << 8) | buf[1];
    mpu->accely = ((uint16_t)buf[2] << 8) | buf[3];
    mpu->accelz = ((uint16_t)buf[4] << 8) | buf[5];
  }
  return res;
  ;
}

/**
 * @brief IIC连续写
 * @param addr 器件地址
 * @param reg 寄存器地址
 * @param len 写入长度
 * @param buf 数据区指针
 * @return 0表示正常，其他值为错误代码
 */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  uint8_t i;
  MPU_IIC_Start();
  MPU_IIC_Send_Byte((addr << 1) | 0);  //发送器件地址+写命令
  if (MPU_IIC_Wait_Ack())              //等待应答
  {
    MPU_IIC_Stop();
    return 1;
  }
  MPU_IIC_Send_Byte(reg);  //写寄存器地址
  MPU_IIC_Wait_Ack();      //等待应答
  for (i = 0; i < len; i++) {
    MPU_IIC_Send_Byte(buf[i]);  //发送数据
    if (MPU_IIC_Wait_Ack())     //等待ACK
    {
      MPU_IIC_Stop();
      return 1;
    }
  }
  MPU_IIC_Stop();
  return 0;
}

/**
 * @brief IIC连续读
 * @param addr 器件地址
 * @param reg 要读取的寄存器地址
 * @param len 要读取的长度
 * @param buf 读取到的数据存储区
 * @return 0表示正常，其他值为错误代码
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  MPU_IIC_Start();
  MPU_IIC_Send_Byte((addr << 1) | 0);  //发送器件地址+写命令
  if (MPU_IIC_Wait_Ack())              //等待应答
  {
    MPU_IIC_Stop();
    return 1;
  }
  MPU_IIC_Send_Byte(reg);  //写寄存器地址
  MPU_IIC_Wait_Ack();      //等待应答
  MPU_IIC_Start();
  MPU_IIC_Send_Byte((addr << 1) | 1);  //发送器件地址+读命令
  MPU_IIC_Wait_Ack();                  //等待应答
  while (len) {
    if (len == 1)
      *buf = MPU_IIC_Read_Byte(0);  //读数据,发送nACK
    else
      *buf = MPU_IIC_Read_Byte(1);  //读数据,发送ACK
    len--;
    buf++;
  }
  MPU_IIC_Stop();  //产生一个停止条件
  return 0;
}

/**
 * @brief IIC写一个字节
 * @param reg 寄存器地址
 * @param data 数据
 * @return 0表示正常，其他值为错误代码
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data) {
  MPU_IIC_Start();
  MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0);  //发送器件地址+写命令
  if (MPU_IIC_Wait_Ack())                  //等待应答
  {
    MPU_IIC_Stop();
    return 1;
  }
  MPU_IIC_Send_Byte(reg);   //写寄存器地址
  MPU_IIC_Wait_Ack();       //等待应答
  MPU_IIC_Send_Byte(data);  //发送数据
  if (MPU_IIC_Wait_Ack())   //等待ACK
  {
    MPU_IIC_Stop();
    return 1;
  }
  MPU_IIC_Stop();
  return 0;
}

/**
 * @brief IIC读一个字节
 * @param reg 寄存器地址
 * @return 读到的数据
 */
uint8_t MPU_Read_Byte(uint8_t reg) {
  uint8_t res;
  MPU_IIC_Start();
  MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0);  //发送器件地址+写命令
  MPU_IIC_Wait_Ack();                      //等待应答
  MPU_IIC_Send_Byte(reg);                  //写寄存器地址
  MPU_IIC_Wait_Ack();                      //等待应答
  MPU_IIC_Start();
  MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1);  //发送器件地址+读命令
  MPU_IIC_Wait_Ack();                      //等待应答
  res = MPU_IIC_Read_Byte(0);              //读取数据,发送nACK
  MPU_IIC_Stop();                          //产生一个停止条件
  return res;
}

/**
 * @brief  通过DMP获取姿态数据并填充到结构体中
 * @param  data: 指向用于存储数据的结构体
 * @retval 0: 成功
 *         1: FIFO读取失败
 *         2: 无四元数数据
 */
uint8_t MPU_DMP_ReadData(MPU* data) {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  unsigned long sensor_timestamp;
  short gyro[3], accel[3], sensors;
  unsigned char more;
  long quat[4];
  uint8_t ret = 0;

  // 从FIFO读取数据
  if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more) == 0) {
    // 检查是否有四元数数据
    if (sensors & INV_WXYZ_QUAT) {
      // 将四元数从q30格式转换为浮点数
      q0 = quat[0] / q30;
      q1 = quat[1] / q30;
      q2 = quat[2] / q30;
      q3 = quat[3] / q30;

      // 通过四元数计算欧拉角并填充到结构体中
      data->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;                                     // pitch
      data->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;      // roll
      data->yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;  // yaw

      // 原始陀螺仪和加速度计数据
      data->gyrox = gyro[0] / 16.4f;  // 转换为°/s，根据量程设置可能需要调整
      data->gyroy = gyro[1] / 16.4f;
      data->gyroz = gyro[2] / 16.4f;
      data->accelx = accel[0] / 16384.0f;  // 转换为g，根据量程设置可能需要调整
      data->accely = accel[1] / 16384.0f;
      data->accelz = accel[2] / 16384.0f;
    } else {
      ret = 2;
    }
  } else {
    ret = 1;
  }

  return ret;
}
