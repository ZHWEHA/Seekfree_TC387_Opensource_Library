#ifndef _CONTROL_H_
#define _CONTROL_H_

//PID参数 ->需根据实车调试
#define PID_KP  1.2f
#define PID_KI  0.1f
#define PID_KD  0.05f

//控制周期，要与PIT定时器匹配
#define CONTROL_DT  0.005f   //5ms

//外部全局目标值
extern double target_speed;      //目标速度百分比 -100~100
extern double target_yaw_rate;   //目标偏转角速度 °/s，正左转

#endif
