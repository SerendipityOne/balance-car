#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"  // 包含HAL库定义，如 uint8_t, int16_t

/* MPU6050 I2C地址 */
#define MPU6050_DEFAULT_ADDRESS 0x68  // AD0引脚接GND时的地址

/* MPU6050 寄存器地址定义 */
#define MPU6050_SELF_TEST_X 0x0D        // X轴自检寄存器
#define MPU6050_SELF_TEST_Y 0x0E        // Y轴自检寄存器
#define MPU6050_SELF_TEST_Z 0x0F        // Z轴自检寄存器
#define MPU6050_SELF_TEST_A 0x10        // 加速度计自检寄存器
#define MPU6050_XG_OFFS_TC 0x00         // X轴陀螺仪温度补偿偏移寄存器
#define MPU6050_YG_OFFS_TC 0x01         // Y轴陀螺仪温度补偿偏移寄存器
#define MPU6050_ZG_OFFS_TC 0x02         // Z轴陀螺仪温度补偿偏移寄存器
#define MPU6050_X_FINE_GAIN 0x03        // X轴陀螺仪细调增益寄存器
#define MPU6050_Y_FINE_GAIN 0x04        // Y轴陀螺仪细调增益寄存器
#define MPU6050_Z_FINE_GAIN 0x05        // Z轴陀螺仪细调增益寄存器
#define MPU6050_XA_OFFS_H 0x06          // X轴加速度计偏移高字节寄存器
#define MPU6050_XA_OFFS_L_TC 0x07       // X轴加速度计偏移低字节和温度补偿寄存器
#define MPU6050_YA_OFFS_H 0x08          // Y轴加速度计偏移高字节寄存器
#define MPU6050_YA_OFFS_L_TC 0x09       // Y轴加速度计偏移低字节和温度补偿寄存器
#define MPU6050_ZA_OFFS_H 0x0A          // Z轴加速度计偏移高字节寄存器
#define MPU6050_ZA_OFFS_L_TC 0x0B       // Z轴加速度计偏移低字节和温度补偿寄存器
#define MPU6050_PRODUCT_ID 0x0C         // 产品ID寄存器
#define MPU6050_XG_OFFS_USRH 0x13       // X轴陀螺仪偏移高字节寄存器
#define MPU6050_XG_OFFS_USRL 0x14       // X轴陀螺仪偏移低字节寄存器
#define MPU6050_YG_OFFS_USRH 0x15       // Y轴陀螺仪偏移高字节寄存器
#define MPU6050_YG_OFFS_USRL 0x16       // Y轴陀螺仪偏移低字节寄存器
#define MPU6050_ZG_OFFS_USRH 0x17       // Z轴陀螺仪偏移高字节寄存器
#define MPU6050_ZG_OFFS_USRL 0x18       // Z轴陀螺仪偏移低字节寄存器
#define MPU6050_XA_OFFS_USRH 0x1A       // X轴加速度计偏移高字节寄存器
#define MPU6050_XA_OFFS_USRL 0x1B       // X轴加速度计偏移低字节寄存器
#define MPU6050_YA_OFFS_USRH 0x1D       // Y轴加速度计偏移高字节寄存器
#define MPU6050_YA_OFFS_USRL 0x1E       // Y轴加速度计偏移低字节寄存器
#define MPU6050_ZA_OFFS_USRH 0x20       // Z轴加速度计偏移高字节寄存器
#define MPU6050_ZA_OFFS_USRL 0x21       // Z轴加速度计偏移低字节寄存器
#define MPU6050_CONFIG 0x1A             // 配置寄存器
#define MPU6050_GYRO_CONFIG 0x1B        // 陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG 0x1C       // 加速度计配置寄存器
#define MPU6050_FIFO_EN 0x23            // FIFO使能寄存器
#define MPU6050_I2C_MST_CTRL 0x24       // I2C主控寄存器
#define MPU6050_INT_PIN_CFG 0x37        // 中断引脚配置寄存器
#define MPU6050_INT_ENABLE 0x38         // 中断使能寄存器
#define MPU6050_DMP_INT_STATUS 0x39     // DMP中断状态寄存器
#define MPU6050_INT_STATUS 0x3A         // 中断状态寄存器
#define MPU6050_ACCEL_XOUT_H 0x3B       // X轴加速度输出高字节寄存器
#define MPU6050_ACCEL_XOUT_L 0x3C       // X轴加速度输出低字节寄存器
#define MPU6050_ACCEL_YOUT_H 0x3D       // Y轴加速度输出高字节寄存器
#define MPU6050_ACCEL_YOUT_L 0x3E       // Y轴加速度输出低字节寄存器
#define MPU6050_ACCEL_ZOUT_H 0x3F       // Z轴加速度输出高字节寄存器
#define MPU6050_ACCEL_ZOUT_L 0x40       // Z轴加速度输出低字节寄存器
#define MPU6050_TEMP_OUT_H 0x41         // 温度输出高字节寄存器
#define MPU6050_TEMP_OUT_L 0x42         // 温度输出低字节寄存器
#define MPU6050_GYRO_XOUT_H 0x43        // X轴角速度输出高字节寄存器
#define MPU6050_GYRO_XOUT_L 0x44        // X轴角速度输出低字节寄存器
#define MPU6050_GYRO_YOUT_H 0x45        // Y轴角速度输出高字节寄存器
#define MPU6050_GYRO_YOUT_L 0x46        // Y轴角速度输出低字节寄存器
#define MPU6050_GYRO_ZOUT_H 0x47        // Z轴角速度输出高字节寄存器
#define MPU6050_GYRO_ZOUT_L 0x48        // Z轴角速度输出低字节寄存器
#define MPU6050_SIGNAL_PATH_RESET 0x68  // 信号路径复位寄存器
#define MPU6050_ACCEL_CONFIG_2 0x1D     // 加速度计配置寄存器2
#define MPU6050_USER_CTRL 0x6A          // 用户控制寄存器
#define MPU6050_PWR_MGMT_1 0x6B         // 电源管理寄存器1
#define MPU6050_PWR_MGMT_2 0x6C         // 电源管理寄存器2
#define MPU6050_FIFO_COUNTH 0x72        // FIFO计数高字节寄存器
#define MPU6050_FIFO_COUNTL 0x73        // FIFO计数低字节寄存器
#define MPU6050_FIFO_R_W 0x74           // FIFO读写寄存器
#define MPU6050_WHO_AM_I 0x75           // 设备ID寄存器

/* 数据结构体定义 */
typedef struct {
  // 原始加速度数据
  int16_t AccX;
  int16_t AccY;
  int16_t AccZ;

  // 原始角速度数据(陀螺仪)
  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;

  // 欧拉角
  float Pitch;  // 俯仰角 Y轴
  float Roll;   // 横滚角 X轴
  float Yaw;    // 偏航角 Z轴
} MPU6050_Data_t;

/* 函数声明 */
void MPU6050_Init(void);
uint8_t MPU6050_ReadID(void);
void MPU6050_ReadData(MPU6050_Data_t* data);

#endif /* __MPU6050_H */
