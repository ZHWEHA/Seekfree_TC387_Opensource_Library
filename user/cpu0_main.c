/*
 *
 * 移动控制
 *
 */

#include<stdint.h>
#include<limits.h>

#include "zf_common_headfile.h"

#include "Process/Process.h"
#include "Data.h"
#include "attitude.h"
#include "control.h"
#include "motor.h"

#pragma section all "cpu0_dsram"

uint16_t Speed = 0;             //当前速度
uint16_t Speed_Mult = 1;
uint16_t Speed_Range = 800;     //速度范围

uint16_t Dire = 0;              //当前方向
uint16_t Dire_Range = 624;      //方向范围
uint16_t Dire_Mult = 1;


int core0_main(void)
{
    /*获取时钟频率<务必保留>*/
    clock_init();
    /*初始化默认调试串口*/
    debug_init();

    /*延时初始*/
    system_delay_init();

    //初始化 PIT 定时器，周期 5ms
    pit_ms_init(CCU60_CH0, 5);

    // 初始化姿态模块（含陀螺仪零偏校准）
    Attitude_Init();

    // 初始化电机模块
    Motor_Start();

    // 初始化控制模块（设置PID参数等）
    Control_Init();


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

//PIT中断服务函数 周期：5ms
void CCU60_CH0_IRQHandler(void){
    pit_clear_flag(CCU60_CH0);

    //姿态更新
    Attitude_Update(0.005f);

    //获取反馈
    float actual_yaw_rate = Attitude_GetYawRate();
    float actual_yaw = Attitude_GetYaw();

    //方向盘控制
    float steering_angle = Get_Steering_Encoder();   //你需要提供

    float steering_input = steering_angle / 30.0f;   //根据实际调整

    Control_Update_Manual(actual_yaw_rate, steering_input);
}
#pragma section all restore