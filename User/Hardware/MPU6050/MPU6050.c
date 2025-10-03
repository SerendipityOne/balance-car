#include "MPU6050.h"
#include <math.h>
#include "Delay.h"
#include "MyI2C.h"

#define MPU6050_ADDR MPU6050_DEFAULT_ADDRESS
#define M_PI 3.14159265358979323846

// MPU6050灵敏度参数
#define MPU6050_ACCEL_SENSITIVITY_16G 2048.0f  // ±16g时的灵敏度 (LSB/g)
#define MPU6050_GYRO_SENSITIVITY_2000 16.4f    // ±2000°/s时的灵敏度 (LSB/°/s)

/* 私有函数声明 */
static void MPU6050_WriteReg(uint8_t reg, uint8_t data);
static uint8_t MPU6050_ReadReg(uint8_t reg);
static void MPU6050_ReadRegs(uint8_t reg, uint8_t len, uint8_t* data);

/* 私有函数实现 */
/**
 * @brief  向MPU6050指定寄存器写入一个字节数据
 * @param  reg: 寄存器地址
 * @param  data: 要写入的数据
 * @retval None
 */
static void MPU6050_WriteReg(uint8_t reg, uint8_t data) {
  MyI2C_Start();
  MyI2C_SendByte(MPU6050_ADDR << 1 | 0);  // 发送写命令 (地址 + W位)
  if (MyI2C_ReceiveAck()) {
    // 处理错误：从机无应答
    MyI2C_Stop();
    return;
  }
  MyI2C_SendByte(reg);  // 发送寄存器地址
  MyI2C_ReceiveAck();
  MyI2C_SendByte(data);  // 发送数据
  MyI2C_ReceiveAck();
  MyI2C_Stop();
}

/**
 * @brief  从MPU6050指定寄存器读取一个字节数据
 * @param  reg: 寄存器地址
 * @retval 读取到的数据
 */
static uint8_t MPU6050_ReadReg(uint8_t reg) {
  uint8_t data;
  MyI2C_Start();
  MyI2C_SendByte(MPU6050_ADDR << 1 | 0);  // 发送写命令，指定寄存器
  if (MyI2C_ReceiveAck()) {
    MyI2C_Stop();
    return 0xFF;  // 返回错误值
  }
  MyI2C_SendByte(reg);
  MyI2C_ReceiveAck();

  MyI2C_Start();                          // 重新发送起始信号
  MyI2C_SendByte(MPU6050_ADDR << 1 | 1);  // 发送读命令 (地址 + R位)
  MyI2C_ReceiveAck();
  data = MyI2C_ReceiveByte();
  MyI2C_SendAck(1);  // 发送NACK，表示读取结束
  MyI2C_Stop();
  return data;
}

/**
 * @brief  从MPU6050指定寄存器连续读取多个字节数据
 * @param  reg: 起始寄存器地址
 * @param  len: 要读取的字节数
 * @param  data: 存放读取数据的缓冲区指针
 * @retval None
 */
static void MPU6050_ReadRegs(uint8_t reg, uint8_t len, uint8_t* data) {
  MyI2C_Start();
  MyI2C_SendByte(MPU6050_ADDR << 1 | 0);  // 发送写命令，指定寄存器
  MyI2C_ReceiveAck();
  MyI2C_SendByte(reg);
  MyI2C_ReceiveAck();

  MyI2C_Start();                          // 重新发送起始信号
  MyI2C_SendByte(MPU6050_ADDR << 1 | 1);  // 发送读命令
  MyI2C_ReceiveAck();

  for (uint8_t i = 0; i < len; i++) {
    data[i] = MyI2C_ReceiveByte();
    if (i == len - 1) {
      MyI2C_SendAck(1);  // 最后一个字节发送NACK
    } else {
      MyI2C_SendAck(0);  // 发送ACK
    }
  }
  MyI2C_Stop();
}

/* 公共函数实现 */

/**
 * @brief  初始化MPU6050
 * @param  None
 * @retval None
 */
void MPU6050_Init(void) {
  MyI2C_Init();   // 初始化I2C引脚
  Delay_ms(100);  // 等待MPU6050上电稳定

  // 复位MPU6050
  MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
  Delay_ms(100);

  // 唤醒MPU6050并设置时钟源
  MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);

  // 配置陀螺仪量程为 ±2000°/s
  MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
  // 配置加速度计量程为 ±16g
  MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
  // 配置数字低通滤波器(DLPF)
  MPU6050_WriteReg(MPU6050_CONFIG, 0x03);

  // I2C主模式关闭
  MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00);
  // 关闭FIFO
  MPU6050_WriteReg(MPU6050_FIFO_EN, 0x00);
  // INT引脚低电平有效，开漏输出
  MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x80);
  // 使能数据就绪中断
  MPU6050_WriteReg(MPU6050_INT_ENABLE, 0x01);
}

/**
 * @brief  读取MPU6050的ID
 * @param  None
 * @retval 返回读取到的ID (应为0x68)
 */
uint8_t MPU6050_ReadID(void) {
  return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief  读取MPU6050传感器数据并计算角度
 * @param  data: 指向用于存储数据的结构体
 * @retval None
 */
void MPU6050_ReadData(MPU6050_Data_t* data) {
  uint8_t buf[14];
  // 从ACCEL_XOUT_H寄存器开始，连续读取14个字节
  MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, 14, buf);

  // 将读取到的字节数据组合成16位数据
  data->AccX = (int16_t)((buf[0] << 8) | buf[1]);
  data->AccY = (int16_t)((buf[2] << 8) | buf[3]);
  data->AccZ = (int16_t)((buf[4] << 8) | buf[5]);

  data->GyroX = (int16_t)((buf[8] << 8) | buf[9]);
  data->GyroY = (int16_t)((buf[10] << 8) | buf[11]);
  data->GyroZ = (int16_t)((buf[12] << 8) | buf[13]);

  // 计算Roll (横滚角) - 绕X轴旋转的角度
  data->Roll = atan2f((float)data->AccY, (float)data->AccZ) * 180.0f / M_PI;

  // 计算Pitch (俯仰角) - 绕Y轴旋转的角度
  data->Pitch = atan2f(-(float)data->AccX, sqrtf((float)data->AccY * data->AccY + (float)data->AccZ * data->AccZ)) * 180.0f / M_PI;

  // Yaw (偏航角) 通常不能仅通过加速度计准确计算，需要磁力计或通过陀螺仪积分
  // 这里简单设为0，实际应用中需要更复杂的传感器融合算法（如卡尔曼滤波或互补滤波）
  data->Yaw = 0.0f;
}
