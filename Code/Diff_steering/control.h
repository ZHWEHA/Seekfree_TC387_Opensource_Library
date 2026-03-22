#ifndef CONTROL_H
#define CONTROL_H

#include "zf_common_headfile.h"

//单位说明
// 角度：度
// 角速度：度/秒
// 时间：秒

//控制周期
#define CONTROL_DT 0.005f //5ms

//内环PID（角速度）
#define YAW_RATE_KP 1.2f
#define YAW_RATE_KI 0.1f
#define YAW_RATE_KD 0.05f

#define PID_OUTPUT_LIMIT 100.0f
#define INTEGRAL_LIMIT 50.0f

//PID结构体
typedef struct {
        float Kp, Ki, Kd;
        float dt;

        float integral;
        float prev_error;

        float out_min, out_max;
        float integral_limit;

        float derivative_lpf;
        float prev_derivative;

} PID_Controller;

//初始化PID控制
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max, float int_limit, float lpf_cutoff);

//更新PID控制
float PID_Update(PID_Controller *pid, float setpoint, float measurement);

//初始化控制模块
void Control_Init(void);

//设置目标速度
//speed 速度百分比 -100~100
void Control_SetTargetSpeed(float speed);

//设置目标航向角
//yaw 航向角 (度)，范围 -180 ~ 180
void Control_SetTargetYaw(float yaw);

//控制更新函数 -> 在定时器中断中周期性调用 （自动模式）
void Control_Update(float actual_yaw_rate, float actual_yaw);

//方向盘控制
//steering_input:-1 ~ 1（方向盘归一化输入）
void Control_Update_Manual(float actual_yaw_rate, float steering_input);

#endif