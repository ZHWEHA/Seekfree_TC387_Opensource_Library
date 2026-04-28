/*
 *
 * 移动控制
 *
 */

//0 与 1 同步锁核
#include<stdint.h>
#include<limits.h>

#include"zf_common_headfile.h"

#include"Data.h"
#include"Process/Process.h"
#include"Function/Function.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    /*获取时钟频率<务必保留>*/
    clock_init();
    /*初始化默认调试串口*/
    debug_init();
    /*延时初始*/
    system_delay_init();
    //Process初始化
    Proc_SE();
    //等待所有核心初始化完毕
    cpu_wait_event_ready();

    //初始化变量
    OperateMode = OM_Wire;       //默认有线驾驶

    while(TRUE)
    {
        switch (OperateMode)
        {
            case OM_Wire:
                Func_Wire();
                break;
            case OM_Remote:
                Func_Remote();
                break;
            case OM_Network:
                Func_Network();
                break;
            case OM_Auto:
                Func_Auto();
                break;
            default:
                OperateMode = OM_Wire;
        }

    }
    return 0;
}

#pragma section all restore
