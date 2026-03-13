/*
 *
 * 移动控制
 *
 */

#include<stdint.h>
#include<limits.h>

#include "zf_common_headfile.h"

#include"Process/Process.h"
#include"Data.h"
#include "attitude.h"
#include "control.h"

#pragma section all "cpu0_dsram"

uint16_t Speed = 0;             //当前速度
uint16_t Speed_Mult = 1;
uint16_t Speed_Range = 800;     //速度范围

uint16_t Dire = 0;              //当前方向
uint16_t Dire_Range = 624;      //方向范围
uint16_t Dire_Mult = 1;

double target_speed = 0;      //目标速度百分比 -100~100
double target_yaw_rate = 0;   //目标偏转角速度 °/s

int core0_main(void)
{
    /*获取时钟频率<务必保留>*/
    clock_init();
    /*初始化默认调试串口*/
    debug_init();

    /*延时初始*/
    system_delay_init();

    Attitude_Init();      // MU初始化及零偏校准
    Motor_Start();        //电机PWM和GPIO初始化

    //PIT定时器，周期5ms
    pit_ms_init(CCU60_CH0, 5);   //选择CCU60通道0，5ms

    /*开始*/
    Start();

    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    OperateMode = Wire;       //默认有线驾驶

    while (TRUE)
    {
        /*有线驾驶处理*/

        /*有线驾驶没有动作，无线驾驶处理*/
        if (OperateState & OS_NoneWire)
        {//允许无线驾驶

        }

        /*无线驾驶没有动作，网络驾驶处理*/
        if (OperateState & OS_Network)
        {//允许网络驾驶

        }

        /*无限驾驶没有动作，自动驾驶处理*/
        if (OperateState & OS_Auto)
        {//允许自动驾驶

        }

    }

    //结束
    End();
    return 0;
}

//PIT中断服务函数
void CCU60_CH0_IRQHandler(void){
    //清除中断标志
    pit_clear_flag(CCU60_CH0);

    //1. 更新姿态数据
    Attitude_Update();

    //2. 获取实际偏航角速度
    float actual_yaw_rate = Attitude_GetYawRate();

    //3. PID计算
    static float integral = 0, prev_error = 0;
    float error = (float)target_yaw_rate - actual_yaw_rate;
    integral += error * CONTROL_DT;               //CONTROL_DT 在 control.h 中定义为 0.005f
    //积分限幅
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    float derivative = (error - prev_error) / CONTROL_DT;
    float output = PID_KP * error + PID_KI * integral + PID_KD * derivative;
    prev_error = error;

    //4. 计算左右轮速度
    double base_speed = target_speed;
    double left_speed  = base_speed - output;
    double right_speed = base_speed + output;

    //限幅到 -100 ~ 100
    if (left_speed > 100.0) left_speed = 100.0;
    if (left_speed < -100.0) left_speed = -100.0;
    if (right_speed > 100.0) right_speed = 100.0;
    if (right_speed < -100.0) right_speed = -100.0;

    //5. 写入电机
    Write_L_Speed(left_speed);
    Write_R_Speed(right_speed);
}

#pragma section all restore
