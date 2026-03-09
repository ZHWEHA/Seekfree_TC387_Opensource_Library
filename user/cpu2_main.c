#include "zf_common_headfile.h"
#pragma section all "cpu2_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

void core2_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);


    cpu_wait_event_ready();
    while (TRUE)
    {


    }
}

#pragma section all restore
