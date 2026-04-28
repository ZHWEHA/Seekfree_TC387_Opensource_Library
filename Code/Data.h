#ifndef _CODE_DATA_INCLUDE_
#define _CODE_DATA_INCLUDE_

#include<stdint.h>

typedef uint8_t STATE;

typedef enum
{
    OM_Wire = 1,    //有线控制
    OM_Remote,      //无线控制
    OM_Network,     //网络控制
    OM_Auto,        //自动控制
} OperateModeEn;

extern OperateModeEn OperateMode;

#define OS_Wire     0x00    //默认手动
#define OS_Remote   0x02
#define OS_Network  0x04
#define OS_Auto     0x08
extern uint8_t OperateState;       //操作可用状态

//自动驾驶任务
#endif
