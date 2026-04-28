#ifndef SERVO_H
#define SERVO_H

#include "tim.h"

// 初始化舵机（设置中位）
void Servo_Init(void);

// 设置俯仰舵机（180°范围，0～180°）
void Servo_Pitch_SetAngle(float angle);

// 设置航向舵机（270°范围，0～270°）
void Servo_Yaw_SetAngle(float angle);

#endif
