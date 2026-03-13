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

#pragma section all restore
