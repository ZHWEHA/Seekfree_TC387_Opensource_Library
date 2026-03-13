#include"zf_device_ips200.h"

typedef enum
{
    RGB565_000 = (0x0000),

    RGB565_008 = (0x000F),
    RGB565_00F = (0x001F),

    RGB565_080 = (0x03E0),
    RGB565_0F0 = (0x07E0),

    RGB565_800 = (0x7800),
    RGB565_F00 = (0xF800),

    RGB565_808 = (0x780F),
    RGB565_F0F = (0xF81F),

    RGB565_880 = (0x7BE0),
    RGB565_FF0 = (0xFFE0),

    RGB565_FFF = (0xFFFF),
} Color_RGB565En;

typedef union
{
        struct
        {
                uint8_t R : 5;
                uint8_t G : 6;
                uint8_t B : 5;
        } RGB;
        uint16 Data;
} Color_RGB565Un;

void D_IPS200_Start();
