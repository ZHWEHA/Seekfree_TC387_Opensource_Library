#include<stdint.h>

typedef enum
{
    Wire,       //有线控制
    NoneWire,   //无线控制
    Network,    //网络控制
    Auto,       //自动控制
} OperateModeEn;

extern OperateModeEn OperateMode;

#define OS_Wire     0x00    //默认手动
#define OS_NoneWire 0x02
#define OS_Network  0x04
#define OS_Auto     0x08
extern uint8_t OperateState;       //操作可用状态

//自动驾驶任务
