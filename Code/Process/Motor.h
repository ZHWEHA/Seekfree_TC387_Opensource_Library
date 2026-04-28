#ifndef _CODE_PROCESS_MOTOR_INCLUDE_
#define _CODE_PROCESS_MOTOR_INCLUDE_

#include<stdint.h>

extern void Proc_Motor_SE();

//反向：< 0；正向：> 0；
extern void Proc_Write_Direction_Speed(double PPercent);

extern void Proc_Write_LR_Speed(double PPercent);
extern void Proc_Write_L_Speed(double PPercent);
extern void Proc_Write_R_Speed(double PPercent);

#endif
