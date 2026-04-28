
//舵机型号为s20f

#include "servo.h"



// 通用角度 → 脉宽转换函数
static uint16_t angle_to_pulse(float angle, float min_angle, float max_angle,
                               uint16_t min_pulse, uint16_t max_pulse) {
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;
    return min_pulse + (uint16_t)((angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle));
}

// 俯仰舵机：180°舵机，脉宽 500~2500us
void Servo_Pitch_SetAngle(float angle) {
    uint16_t pulse = angle_to_pulse(angle, 0.0f, 180.0f, 500, 2500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pulse);
}

// 航向舵机：270°舵机，脉宽 500~2500us
void Servo_Yaw_SetAngle(float angle) {
    uint16_t pulse = angle_to_pulse(angle, 0.0f, 270.0f, 500, 2500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pulse);
}

void Servo_Init(void) {
    Servo_Pitch_SetAngle(90.0f);   // 中位 90°
    Servo_Yaw_SetAngle(135.0f);    // 中位 135°
}

