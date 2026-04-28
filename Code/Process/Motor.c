#include"Motor.h"
#include<math.h>

#include "zf_common_headfile.h"

double Curve_Mult(double x)
{
    double M = 1.2;
    double N = 2;

    double t = x - 1.0;
    double base = 1.0 - pow(t * t, M);
    return pow(base, N);
}

#define PIN_Dire_GPIO       (P21_2)
#define PIN_Dire_PWM        (ATOM0_CH1_P21_3)
#define PIN_Void_Dire_GPIO  (P21_4)               //悬空
#define PIN_Void_Dire_PWM   (ATOM0_CH3_P21_5)     //悬空

#define PIN_L_GPIO          (P02_6)
#define PIN_L_PWM           (ATOM0_CH5_P02_5)
#define PIN_R_GPIO          (P02_4)
#define PIN_R_PWM           (ATOM0_CH7_P02_7)

extern void Proc_Motor_SE()
{
    const uint32_t Freq = 17000;
    //方向
    gpio_init(PIN_Dire_GPIO, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PIN_Dire_PWM, Freq, 0);

    gpio_init(PIN_Void_Dire_GPIO, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PIN_Void_Dire_PWM, Freq, 0);
    //左轮
    gpio_init(PIN_L_GPIO, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PIN_L_PWM, Freq, 0);
    //右轮
    gpio_init(PIN_R_GPIO, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PIN_R_PWM, Freq, 0);

}

//方向
extern void Proc_Write_Direction_Speed(double PPercent)
{
    //控制输出正负电压
    if (PPercent < 0)
        gpio_set_level(PIN_Dire_GPIO, GPIO_LOW);
    else
        gpio_set_level(PIN_Dire_GPIO, GPIO_HIGH);
    //写入电压
    pwm_set_duty(PIN_Dire_PWM, fabs(PPercent) * PWM_DUTY_MAX);
}

//速度
extern void Proc_Write_LR_Speed(double PPercent)
{
    //控制输出正负电压
    if (PPercent < 0)
    {
        gpio_set_level(PIN_L_GPIO, GPIO_LOW);
        gpio_set_level(PIN_R_GPIO, GPIO_LOW);
    }
    else
    {
        gpio_set_level(PIN_L_GPIO, GPIO_HIGH);
        gpio_set_level(PIN_R_GPIO, GPIO_HIGH);
    }
    //写入电压
    pwm_set_duty(PIN_L_PWM, PPercent * PWM_DUTY_MAX);
    pwm_set_duty(PIN_R_PWM, PPercent * PWM_DUTY_MAX);
}
extern void Proc_Write_L_Speed(double PPercent)
{
    //控制输出正负电压
    if (PPercent < 0)
        gpio_set_level(PIN_L_GPIO, GPIO_LOW);
    else
        gpio_set_level(PIN_L_GPIO, GPIO_HIGH);
    //写入电压
    pwm_set_duty(PIN_L_PWM, fabs(PPercent) * PWM_DUTY_MAX);
}
extern void Proc_Write_R_Speed(double PPercent)
{
    //控制输出正负电压
    if (PPercent < 0)
        gpio_set_level(PIN_R_GPIO, GPIO_LOW);
    else
        gpio_set_level(PIN_R_GPIO, GPIO_HIGH);
    //写入电压
    pwm_set_duty(PIN_R_PWM, fabs(PPercent) * PWM_DUTY_MAX);
}
