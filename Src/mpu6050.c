#include "mpu6050.h"
#include "i2c.h"   // 使用 hi2c1
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


static I2C_HandleTypeDef *mpu_i2c = &hi2c1;

// 写寄存器
static void MPU6050_WriteReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// 读多个寄存器
static void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void MPU6050_Init(void) {
	//复位-----MPU6050_PWR_MGMT_1为0x6b---为寄存器地址
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	
	// 退出睡眠模式---进入唤醒模式
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);


	// 加速度计 ±8g   (0x10)    ±2g（0x00）  g为单位表示多少倍的重力加速度
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x10);
	
	// 陀螺仪 ±2000 °/s (0x18)----MPU6050_GYRO_CONFIG--0x1b寄存器地址
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
}


//本函数获取生数据，即为处理的数据
void MPU6050_Read_Raw(int16_t *ax, int16_t *ay, int16_t *az,int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[14];
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 14);
	
	
	  //所有数据均为带符号数整型
	  //加速度，温度和陀螺仪数据地址连续，以字节为单位，从0x3b到0x48
	  //高位左移八位跟低位进行拼接，buf[0]~buf[5]存储的是加速度数据
	  //地址为0x3b到0x40一共8字节
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
	
    // 跳过温度数据 (buf[6], buf[7])---- 地址为0x41,0x42存储温度数据 共两字节
	
	  // buf[8]~buf[13]存储的是陀螺仪数据,地址为0x43到0x48存储陀螺仪数据一共6字节
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}



// 互补滤波得到欧拉角
//横滚角 roll、俯仰角 pitch、偏航角 yaw
void MPU6050_Get_Angles(float *roll, float *pitch, float *yaw) {
	
    static float roll_angle = 0, pitch_angle = 0, yaw_angle = 0;
	
    static uint32_t last_time = 0;
	
    int16_t ax, ay, az, gx, gy, gz;
	
    float dt = 0.01f;

    MPU6050_Read_Raw(&ax, &ay, &az, &gx, &gy, &gz);

    // 加速度计计算俯仰和横滚（单位：度）
	  //俯仰角的计算公式----Pitch = arctan( -ax / √(ay2 + az2) )
    float accel_pitch = atan2f(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
	
	  //横滚角的计算公式----Roll = arctan( ay / az )
    float accel_roll  = atan2f(ay, az) * 180.0f / M_PI;

    // 陀螺仪转换（±2000°/s -> 除以 16.4 得到 °/s）
	  //真实角速度（°/s）= 原始数字 ÷ 16.4----具体看使用手册
    float gyro_pitch = gy / 16.4f;
    float gyro_roll  = gx / 16.4f;
    float gyro_yaw   = gz / 16.4f;

    // 计算时间差
    uint32_t now = HAL_GetTick();
    if (last_time != 0) {
        dt = (now - last_time) / 1000.0f;
        if (dt > 0.05f) dt = 0.02f;
    }
    last_time = now;

    // 互补滤波系数
	  //系数 alpha
    //alpha = 0.96：陀螺仪占 96%
    //1 - alpha = 0.04：加速度计占 4%
		//最终角度 = 陀螺仪预测角度 × 0.96 + 加速度计矫正角度 × 0.04
		//陀螺仪角度 = 上一次角度 + 角速度 × 时间
    float alpha = 0.96f;
    roll_angle  = alpha * (roll_angle  + gyro_roll  * dt) + (1 - alpha) * accel_roll;
    pitch_angle = alpha * (pitch_angle + gyro_pitch * dt) + (1 - alpha) * accel_pitch;
		
		//加速度计 测不出偏航角 yaw所以不需要互补滤波
    yaw_angle  += gyro_yaw * dt;   // 无磁力计修正，会漂移，但对于体感跟随短期可用

    *roll  = roll_angle;
    *pitch = pitch_angle;
    *yaw   = yaw_angle;
}


