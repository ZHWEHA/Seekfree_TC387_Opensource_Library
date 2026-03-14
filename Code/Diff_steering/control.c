#include "control.h"
#include "motor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 静态变量 -> 目标值和 PID 控制器实例
static float target_speed = 0.0f;          //目标速度，范围 -100~100
static float target_yaw = 0.0f;            //目标航向角，单位 度，范围 -180~180
static PID_Controller yaw_angle_pid;       //角度外环 PID
static PID_Controller yaw_rate_pid;        //角速度内环 PID

/**
 * 初始化PID控制
 * pid         PID 结构体指针
 * Kp          比例系数
 * Ki          积分系数
 * Kd          微分系数
 *dt          采样周期  秒
 *out_min     输出最小值
 *out_max     输出最大值
 *int_limit   积分限幅值
 *lpf_cutoff  微分低通滤波截止频率（Hz），若 <= 0 则不滤波
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max, float int_limit, float lpf_cutoff) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = int_limit;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;

    if (lpf_cutoff > 0.0f && dt > 0.0f) {
        float tau = 1.0f / (2.0f * M_PI * lpf_cutoff);
        pid->derivative_lpf = dt / (dt + tau);
    } else {
        pid->derivative_lpf = 0.0f;
    }
}


//PID控制，返回控制输出
float PID_Update(PID_Controller *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    //比例项
    float P = pid->Kp * error;

    //积分项（带限幅）
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;

    //微分项
    float derivative = (error - pid->prev_error) / pid->dt;
    float D;
    if (pid->derivative_lpf > 0.0f) {
        D = pid->derivative_lpf * derivative + (1.0f - pid->derivative_lpf) * pid->prev_derivative;
        pid->prev_derivative = D;
    } else {
        D = derivative;
    }
    D *= pid->Kd;

    pid->prev_error = error;

    float output = P + I + D;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}


//初始化控制模块

void Control_Init(void) {
    //初始化角度外环PID       需调参
    PID_Init(&yaw_angle_pid,
             2.0f,          // Kp
             0.0f,          // Ki,正常情况下不加积分
             0.0f,          // Kd
             CONTROL_DT,
             -200.0f,       //输出限幅：期望角速度 ±200 度/秒
             200.0f,
             50.0f,         //积分限幅 -> 无积分时无效
             0.0f);         //无微分滤波

    //初始化角速度内环PID    无需调参
    PID_Init(&yaw_rate_pid,
             DEFAULT_PID_KP,
             DEFAULT_PID_KI,
             DEFAULT_PID_KD,
             CONTROL_DT,
             -PID_OUTPUT_LIMIT,
             PID_OUTPUT_LIMIT,
             INTEGRAL_LIMIT,
             10.0f);        //微分低通滤波 10Hz

    target_speed = 0.0f;
    target_yaw = 0.0f;
}

//设置目标速度（自动限幅）
void Control_SetTargetSpeed(float speed) {
    if (speed > 100.0f) speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;
    target_speed = speed;
}

//设置目标航向角 ->归一化到 -180~180
void Control_SetTargetYaw(float yaw) {
    //将角度归一化
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    target_yaw = yaw;
}

//控制更新函数 -> 在定时器中断中周期性调用
//actual_yaw_rate 当前实际偏航角速度 单位：度/秒
//actual_yaw      当前实际航向角 单位：度
void Control_Update(float actual_yaw_rate, float actual_yaw) {
    //1. 角度外环：根据目标航向角和实际航向角计算期望角速度
    float desired_yaw_rate = PID_Update(&yaw_angle_pid, target_yaw, actual_yaw);

    //2. 角速度内环：根据期望角速度和实际角速度计算转向输出
    float output = PID_Update(&yaw_rate_pid, desired_yaw_rate, actual_yaw_rate);

    //3. 差速模型
    float left_speed  = target_speed - output;
    float right_speed = target_speed + output;

    //4. 限幅到电机允许范围
    if (left_speed > 100.0f) left_speed = 100.0f;
    if (left_speed < -100.0f) left_speed = -100.0f;
    if (right_speed > 100.0f) right_speed = 100.0f;
    if (right_speed < -100.0f) right_speed = -100.0f;

    //5. 输出到电机
    Write_L_Speed((double)left_speed);
    Write_R_Speed((double)right_speed);
}
