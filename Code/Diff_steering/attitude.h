//姿态处理
#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "zf_common_headfile.h"

//初始化姿态模块
void Attitude_Init(void);

//更新姿态数据
void Attitude_Update(void);

//获取偏转角速度速度，正值表示左转
float Attitude_GetYawRate(void);

// 获取融合后的偏转角，范围 -180~180
float Attitude_GetYaw(void);

#endif
