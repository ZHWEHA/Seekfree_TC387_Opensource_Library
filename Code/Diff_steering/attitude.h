//姿态处理
#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_
#include "zf_common_headfile.h"

/*坐标系定义
右手系
X：前
Y：左
Z：上
*/

//传感器方向修正
#define ATTITUDE_ACC_SIGN_X 1.0f
#define ATTITUDE_ACC_SIGN_Y -1.0f
#define ATTITUDE_ACC_SIGN_Z -1.0f

#define ATTITUDE_GYR_SIGN_X 1.0f
#define ATTITUDE_GYR_SIGN_Y 1.0f
#define ATTITUDE_GYR_SIGN_Z 1.0f

#define ATTITUDE_MAG_SIGN_X 1.0f
#define ATTITUDE_MAG_SIGN_Y -1.0f
#define ATTITUDE_MAG_SIGN_Z -1.0f

//函数接口
//初始化
void Attitude_Init(void);

//周期更新 dt 单位s
void Attitude_Update(float dt);

//姿态角 度
float Attitude_GetYaw(void); //-180 ~ 180

//获取融合后俯仰角
float Attitude_GetPitch(void);

//获取融合后横滚角
float Attitude_GetRoll(void);

//偏航角速度 度/s
float Attitude_GetYawRate(void);

// 磁力计校准
void Attitude_CalibrateMag(uint16_t samples);

#endif