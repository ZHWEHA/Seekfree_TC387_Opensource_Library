#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

//PID参数 -> 根据车况修改
#define CONTROL_DT          0.005f   //控制周期 5ms
#define DEFAULT_PID_KP      1.2f        //内环比例 角速度环
#define DEFAULT_PID_KI      0.1f          //内环积分
#define DEFAULT_PID_KD      0.05f      //内环微分
#define PID_OUTPUT_LIMIT    100.0f   //内环输出限幅
#define INTEGRAL_LIMIT      50.0f    //内环积分限幅

//PID控制结构体
typedef struct {
        float Kp, Ki, Kd;
        float dt;
        float integral;
        float prev_error;
        float out_min, out_max;      //输出限幅
        float integral_limit;        //积分限幅
        float derivative_lpf;        //微分低通滤波系数（0~1，0表示无滤波）
        float prev_derivative;       //上一帧微分值（用于滤波）
}PID_Controller;

/*
 * 初始化PID控制；
 * pid      PID结构体指针
 * Kp       比例系数
 * Ki        积分系数
 * Kd       微分系数
 * dt        采样时间（秒）
 * out_min       输出最小值
 * out_max       输出最大值
 * int_limit        积分限幅
 * lpf_cutoff      微分低通截止频率（Hz），若<=0则不滤波
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max, float int_limit, float lpf_cutoff);

/*
 * 跟新PID控制
 * pid          PID结构体指针
 * setpoint             设定值
 * measurement      测量值
 * return                控制输出
 */
float PID_Update(PID_Controller *pid, float setpoint, float measurement);

//初始化控制模块
void Control_Init(void);

//设置目标速度
//speed 速度百分比 -100~100
void Control_SetTargetSpeed(float speed);

//设置目标航向角
//yaw 航向角 (度)，范围 -180 ~ 180
void Control_SetTargetYaw(float yaw);

//控制更新函数 -> 在定时器中断中周期性调用
//actual_yaw_rate 当前实际偏航角速度 单位：度/秒
//actual_yaw      当前实际航向角 单位：度
void Control_Update(float actual_yaw_rate, float actual_yaw);

#endif
