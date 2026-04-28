#include"Function.h"
#include"zf_common_headfile.h"

#define GPIO_PIN_LEFT           (P20_6)     //◊ÛÃ§∞Â
#define GPIO_PIN_RIGHT          (P20_7)     //”“Ã§∞Â
#define GPIO_PIN_SWITCH_00      (P11_2)     //ø™πÿ_00
#define GPIO_PIN_SWITCH_01      (P11_3)     //ø™πÿ_01
#define GPIO_PIN_SCAN_KEY_00    (P0_0)     //æÿ’ÛR_0
#define GPIO_PIN_SCAN_KEY_01    (P0_0)     //æÿ’ÛR_1
#define GPIO_PIN_SCAN_KEY_02    (P0_0)     //æÿ’ÛR_2
#define GPIO_PIN_SCAN_KEY_03    (P0_0)     //æÿ’ÛR_3
#define GPIO_PIN_SCAN_KEY_10    (P0_0)     //æÿ’ÛC_0
#define GPIO_PIN_SCAN_KEY_11    (P0_0)     //æÿ’ÛC_1
#define GPIO_PIN_SCAN_KEY_12    (P0_0)     //æÿ’ÛC_2
#define GPIO_PIN_SCAN_KEY_13    (P0_0)     //æÿ’ÛC_3

extern void Proc_Function_SE()
{
    gpio_init(GPIO_PIN_LEFT, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(GPIO_PIN_RIGHT, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(GPIO_PIN_SWITCH_00, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(GPIO_PIN_SWITCH_01, GPI, GPIO_HIGH, GPI_PULL_UP);
}

extern uint8_t Proc_Function_Left_State()
{
    return !gpio_get_level(GPIO_PIN_LEFT);
}

extern uint8_t Proc_Function_Right_State()
{
    return !gpio_get_level(GPIO_PIN_RIGHT);
}

extern uint8_t Proc_Function_Switch_00_State()
{
    return !gpio_get_level(GPIO_PIN_SWITCH_00);
}
extern uint8_t Proc_Function_Switch_01_State()
{
    return !gpio_get_level(GPIO_PIN_SWITCH_01);
}



