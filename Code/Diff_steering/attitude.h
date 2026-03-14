//姿态处理
#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_
#include "zf_common_headfile.h"

//传感器坐标符号
#define ATTITUDE_ACC_SIGN_X   1.0f
#define ATTITUDE_ACC_SIGN_Y  -1.0f
#define ATTITUDE_ACC_SIGN_Z  -1.0f

#define ATTITUDE_GYR_SIGN_X   1.0f
#define ATTITUDE_GYR_SIGN_Y   1.0f
#define ATTITUDE_GYR_SIGN_Z   1.0f

#define ATTITUDE_MAG_SIGN_X   1.0f
#define ATTITUDE_MAG_SIGN_Y  -1.0f
#define ATTITUDE_MAG_SIGN_Z  -1.0f

//初始化姿态模块
void Attitude_Init(void);

//周期性跟新姿态数据 -> 与定时器中断同步
//实际采样时间dt由调用提供
void Attitude_Init(void);

//获取融合后偏航角 -> 范围 -180~180
float Attitude_GetYaw(void);

//获取融合后俯仰角
float Attitude_GetPitch(void);

//获取融合后横滚角
float Attitude_GetRoll(void);

//获取当前偏航角 -> 正值为左转      单位：度/秒
float Attitude_GetYawRate(void);

//获取磁力校准
void Attitude_CalibrateMag(uint16_t samples);

#endif
