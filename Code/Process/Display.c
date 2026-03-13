#include "zf_common_headfile.h"

#include<stdint.h>
#include"Display.h"

//#define IPS200_TYPE (IPS200_TYPE_PARALLEL8)     //并行
#define IPS200_TYPE (IPS200_TYPE_SPI)           //IPS

void D_IPS200_Start()
{
    //竖屏模式
    ips200_set_dir(IPS200_PORTAIT);
    //设置字体
    ips200_set_font(IPS200_8X16_FONT);
    //顶景：白；底景：黑；
    ips200_set_color(RGB565_FFF, RGB565_000);
    //初始化为 IPS
    ips200_init(IPS200_TYPE);
}

