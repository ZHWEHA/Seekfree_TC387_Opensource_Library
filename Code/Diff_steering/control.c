#include "control.h"
#include "motor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//静态变量
static float target_speed = 0.0f;
static float target_yaw = 0.0f;

static PID_Controller yaw_angle_pid;
static PID_Controller yaw_rate_pid;

//工具函数
//角度误差（处理 -180~180 环绕）
float Angle_Error(float target, float current){
    float error = target - current;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;

}

//PID初始化
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max, float int_limit, float lpf_cutoff){

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

    if (lpf_cutoff > 0.0f && dt > 0.0f){
        float tau = 1.0f / (2.0f * M_PI * lpf_cutoff);
        pid->derivative_lpf = dt / (dt + tau);
    } else {
        pid->derivative_lpf = 0.0f;
    }

}

//PID更新
float PID_Update(PID_Controller *pid, float setpoint, float measurement){
    if (pid->dt <= 0.0f) return 0.0f; //防止除0

    float error = setpoint - measurement;

    //比例
    float P = pid->Kp * error;

    //积分 带限幅 + 简易anti-windup
    pid->integral += error * pid->dt;

    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    float I = pid->Ki * pid->integral;

    //微分
    float derivative = (error - pid->prev_error) / pid->dt;
    float D;

    if (pid->derivative_lpf > 0.0f){
        D = pid->derivative_lpf * derivative +
            (1.0f - pid->derivative_lpf) * pid->prev_derivative;
        pid->prev_derivative = D;
    } else{
        D = derivative;
    }

    D *= pid->Kd;

    pid->prev_error = error;

    //输出
    float output = P + I + D;

    //输出限幅
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;

}

//控制初始化
void Control_Init(void){
    //外环：角度 -> 角速度
    PID_Init(&yaw_angle_pid, 2.0f, 0.0f, 0.0f, CONTROL_DT, -200.0f, 200.0f, 50.0f, 0.0f);

    //内环：角速度 -> 电机输出
    PID_Init(&yaw_rate_pid,
            YAW_RATE_KP,
            YAW_RATE_KI,
            YAW_RATE_KD,
            CONTROL_DT,
            -PID_OUTPUT_LIMIT,
            PID_OUTPUT_LIMIT,
            INTEGRAL_LIMIT,
            10.0f);

    target_speed = 0.0f;
    target_yaw = 0.0f;

}

//设置目标
void Control_SetTargetSpeed(float speed){
    if (speed > 100.0f) speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;

    target_speed = speed;

}

void Control_SetTargetYaw(float yaw){
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;

    target_yaw = yaw;

}

//================ 自动模式================
//控制更新
void Control_Update(float actual_yaw_rate, float actual_yaw){
    //1.角度误差
    float yaw_error = Angle_Error(target_yaw, actual_yaw);

    //2.外环
    float desired_yaw_rate = PID_Update(&yaw_angle_pid, 0.0f, -yaw_error);

    //3.内环
    float output = PID_Update(&yaw_rate_pid, desired_yaw_rate, actual_yaw_rate);

    //4.差速
    float left_speed  = target_speed - output;
    float right_speed = target_speed + output;

    //5.限幅
    if (left_speed > 100.0f) left_speed = 100.0f;
    if (left_speed < -100.0f) left_speed = -100.0f;

    if (right_speed > 100.0f) right_speed = 100.0f;
    if (right_speed < -100.0f) right_speed = -100.0f;

    //6.输出
    Write_L_Speed(left_speed);
    Write_R_Speed(right_speed);

}

//================ 手动驾驶模式=================
void Control_Update_Manual(float actual_yaw_rate, float steering_input){
    //1.死区（防抖）
    if (fabsf(steering_input) < 0.05f)
        steering_input = 0.0f;

    //2.方向盘 → 目标角速度（核心）
    float target_yaw_rate = steering_input * 150.0f;   // ?可调（转向灵敏度）

    //3.角速度内环
    float output = PID_Update(&yaw_rate_pid,
                              target_yaw_rate,
                              actual_yaw_rate);

    //4.差速
    float left_speed  = target_speed - output;
    float right_speed = target_speed + output;

    //5.限幅
    if (left_speed > 100.0f) left_speed = 100.0f;
    if (left_speed < -100.0f) left_speed = -100.0f;

    if (right_speed > 100.0f) right_speed = 100.0f;
    if (right_speed < -100.0f) right_speed = -100.0f;

    //6.输出
    Write_L_Speed(left_speed);
    Write_R_Speed(right_speed);
}