#include"Function.h"

#include"Process/Process.h"

double Func_Move_Speed_Ratio = 0.1;

extern void Func_Wire()
{
    //刹车
    if(!Proc_Function_Left_State())
    {
        Proc_Write_LR_Speed(0);
        return;     //刹车按下后不处理其他
    }
    //油门
    if(!Proc_Function_Right_State())
    {
        Proc_Write_LR_Speed(Func_Move_Speed_Ratio);
    }
}
extern void Func_Remote()
{

}
extern void Func_Network()
{

}
extern void Func_Auto()
{

}
