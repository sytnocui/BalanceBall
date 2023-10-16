#ifndef _oled_h_
#define _oled_h_

#include "oled_fonts.h"
#include "main.h"
#include "gpio.h"
#include "spi.h"

#define OLED_BRIGHTNESS                 (0xff)                                  // 设置OLED亮度 越大越亮 范围0-0XFF
#define OLED_DEFAULT_DISPLAY_DIR        (OLED_CROSSWISE)                        // 默认的显示方向
#define OLED_DEFAULT_DISPLAY_FONT       (OLED_8X16_FONT )                        // 默认的字体模式

#define OLED_RES(x)                     HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, x)
#define OLED_DC(x)                      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, x)
typedef enum
{
    OLED_CROSSWISE                      = 0,                                    // 横屏模式
    OLED_CROSSWISE_180                  = 1,                                    // 横屏模式  旋转180
}oled_dir_enum;

typedef enum
{
    OLED_6X8_FONT                       = 0,                                    // 6x8      字体
    OLED_8X16_FONT                      = 1,                                    // 8x16     字体
    OLED_16X16_FONT                     = 2,                                    // 16x16    字体 目前不支持
}oled_font_size_enum;

#define OLED_X_MAX                      (128)
#define OLED_Y_MAX                      (64 )

void    oled_clear                      (void);
void    oled_full                       (const uint8_t color);
void    oled_set_dir                    (oled_dir_enum dir);
void    oled_set_font                   (oled_font_size_enum font);
void    oled_draw_point                 (uint16_t x, uint16_t y, const uint8_t color);

void    oled_show_string                (uint16_t x, uint16_t y, const char ch[]);
//void    oled_show_int                   (uint16_t x, uint16_t y, const int32_t dat, uint8_t num);
//void    oled_show_uint                  (uint16_t x, uint16_t y, const uint32_t dat, uint8_t num);
void    oled_show_float                 (uint16_t x, uint16_t y, const double dat, uint8_t num, uint8_t pointnum);

void    oled_init                       (void);

#endif