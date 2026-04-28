#ifndef _CODE_PROCESS_FUNCTION_INCLUDE_
#define _CODE_PROCESS_FUNCTION_INCLUDE_

#include<stdint.h>

//初始化
extern void Proc_Function_SE();

//左踏板状态
//0：左踏板未启用；1：左踏板启用；
extern uint8_t Proc_Function_Left_State();
//右踏板状态
//0：右踏板未启用；1：右踏板启用；
extern uint8_t Proc_Function_Right_State();

//油门状态
//0：油门未启用；1：油门启用；
extern uint8_t Proc_Function_Switch_00_State();
//油门状态
//0：油门未启用；1：油门启用；
extern uint8_t Proc_Function_Switch_01_State();

#endif
