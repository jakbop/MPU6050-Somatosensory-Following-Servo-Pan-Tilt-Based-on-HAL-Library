#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

#define MPU6050_ADDR         0x68 << 1   // I2C 7位地址 0x68，左移一位得到8位写地址
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_XOUT_H 0x3B

// 初始化 MPU6050
void MPU6050_Init(void);

// 读取原始加速度和角速度数据
void MPU6050_Read_Raw(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz);

// 获取欧拉角（俯仰、横滚、偏航）单位：度
void MPU6050_Get_Angles(float *roll, float *pitch, float *yaw);

#endif
