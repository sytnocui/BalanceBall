#include <memory.h>
#include "oled.h"

static uint8_t rx;
static oled_dir_enum        oled_display_dir    = OLED_DEFAULT_DISPLAY_DIR;
static oled_font_size_enum  oled_display_font   = OLED_DEFAULT_DISPLAY_FONT;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写8位数据
// 参数说明     data            数据
// 返回参数     void
// 使用示例     oled_write_data(color);
// 备注信息     内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void oled_write_data (uint8_t data)
{
    OLED_DC(1);
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, 55);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写命令
// 参数说明     cmd             命令
// 返回参数     void
// 使用示例     oled_write_command(0xb0 + y);
// 备注信息     内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void oled_write_command (uint8_t command)
{
    OLED_DC(0);
    HAL_SPI_TransmitReceive(&hspi2, &command, &rx, 1, 55);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 清屏函数
// 参数说明     void
// 返回参数     void
// 使用示例     oled_clear();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_clear (void)
{
    uint8_t y = 0, x = 0;

//    OLED_CS(0);
    for(y = 0; 8 > y; y ++)
    {
        oled_write_command(0xb0 + y);
        oled_write_command(0x01);
        oled_write_command(0x10);
        for(x = 0; OLED_X_MAX > x; x ++)
        {
            oled_write_data(0x00);
        }
    }
//    OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 屏幕填充函数
// 参数说明     color           填充颜色选着(0x00 or 0xff)
// 返回参数     void
// 使用示例     oled_full(0x00);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_full (const uint8_t color)
{
    uint8_t y = 0, x = 0;

//    OLED_CS(0);
    for(y = 0; 8 > y; y ++)
    {
        oled_write_command(0xb0 + y);
        oled_write_command(0x01);
        oled_write_command(0x10);
        for(x = 0; OLED_X_MAX > x; x ++)
        {
            oled_write_data(color);
        }
    }
//    OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED显示坐标设置
// 参数说明     x               x轴坐标设置0-127
// 参数说明     y               y轴坐标设置0-7
// 返回参数     void
// 使用示例     oled_set_coordinate(x, y);
// 备注信息     内部使用用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void oled_set_coordinate (uint8_t x, uint8_t y)
{
    oled_write_command(0xb0 + y);
    oled_write_command(((x & 0xf0) >> 4) | 0x10);
    oled_write_command((x & 0x0f) | 0x00);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示方向
// 参数说明     dir             显示方向  参照oled.h 内 oled_dir_enum 枚举体定义
// 返回参数     void
// 使用示例     oled_set_dir(OLED_CROSSWISE);
// 备注信息     这个函数只有在初始化屏幕之前调用才生效
//-------------------------------------------------------------------------------------------------------------------
void oled_set_dir (oled_dir_enum dir)
{
    oled_display_dir = dir;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示字体
// 参数说明     dir             显示方向  参照 zf_device_oled.h 内 oled_font_size_enum 枚举体定义
// 返回参数     void
// 使用示例     oled_set_font(OLED_8x16_FONT);
// 备注信息     字体可以随时自由设置 设置后生效 后续显示就是新的字体大小
//-------------------------------------------------------------------------------------------------------------------
void oled_set_font (oled_font_size_enum font)
{
    oled_display_font = font;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画点函数
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     color           8 个点数据
// 返回参数     void
// 使用示例     oled_draw_point(0, 0, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_draw_point (uint16_t x, uint16_t y, const uint8_t color)
{
//    OLED_CS(0);
    oled_set_coordinate(x, y);
    oled_write_command(0xb0 + y);
    oled_write_command(((x & 0xf0) >> 4) | 0x10);
    oled_write_command((x & 0x0f) | 0x00);
    oled_write_data(color);
//    OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 显示字符串
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     ch[]            字符串
// 返回参数     void
// 使用示例     oled_show_string(0, 0, "SEEKFREE");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_show_string (uint16_t x, uint16_t y, const char ch[])
{
//    OLED_CS(0);
    uint8_t c = 0, i = 0, j = 0;
    while ('\0' != ch[j])
    {
        switch(oled_display_font)
        {
            case OLED_6X8_FONT:
            {
                c = ch[j] - 32;
                if(x > 126)
                {
                    x = 0;
                    y ++;
                }
                oled_set_coordinate(x, y);
                for(i = 0; 6 > i; i ++)
                {
                    oled_write_data(ascii_font_6x8[c][i]);
                }
                x += 6;
                j ++;
            }break;
            case OLED_8X16_FONT:
            {
                c = ch[j] - 32;
                if(x > 120)
                {
                    x = 0;
                    y ++;
                }
                oled_set_coordinate(x, y);
                for(i = 0; i < 8; i ++)
                {
                    oled_write_data(ascii_font_8x16[c][i]);
                }

                oled_set_coordinate(x, y + 1);
                for(i = 0; i < 8; i ++)
                {
                    oled_write_data(ascii_font_8x16[c][i + 8]);
                }
                x += 8;
                j ++;
            }break;
            case OLED_16X16_FONT:
            {
                // 暂不支持
            }break;
        }
    }
//    OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_double_to_str(data_buffer, 3.1415, 2);                     // 结果输出 data_buffer = "3.14"
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void func_double_to_str (char *str, double number, uint8_t point_bit)
{
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[12];                                                          // 整数字符缓冲
    int data_temp_point[9];                                                     // 小数字符缓冲
    uint8_t bit = point_bit;                                                      // 转换精度位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != bit)
        {
#define     func_abs(x)             ((x) >= 0 ? (x): -(x))
            *str ++ = (func_abs(data_temp[bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            bit --;
        }

        // 小数部分转为字符串
        if(point_bit != 0)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
                *str = '0';
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;
                }
                while(0 != bit)
                {
                    *str ++ = (func_abs(data_temp_point[bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 显示浮点数 (去除整数部分无效的0)
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     dat             需要显示的变量 数据类型 float
// 参数说明     num             整数位显示长度   最高8位
// 参数说明     pointnum        小数位显示长度   最高6位
// 返回参数     void
// 使用示例     oled_show_float(0, 0, x, 2, 3);                 // 显示浮点数   整数显示2位   小数显示三位
// 备注信息     特别注意当发现小数部分显示的值与你写入的值不一样的时候，
//              可能是由于浮点数精度丢失问题导致的，这并不是显示函数的问题，
//              有关问题的详情，请自行百度学习   浮点数精度丢失问题。
//              负数会显示一个 ‘-’号
//-------------------------------------------------------------------------------------------------------------------
void oled_show_float (uint16_t x,uint16_t y,const double dat,uint8_t num,uint8_t pointnum)
{
    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17];
    memset(data_buffer, 0, 17);
    memset(data_buffer, ' ', num + pointnum + 2);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    oled_show_string(x, y, data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     oled_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_init (void)
{
    oled_set_dir(oled_display_dir);

//    OLED_CS(0);
    OLED_RES(0);
    HAL_Delay(50);
    OLED_RES(1);

    oled_write_command(0xae);                                                   // --turn off oled panel
    oled_write_command(0x00);                                                   // ---set low column address
    oled_write_command(0x10);                                                   // ---set high column address
    oled_write_command(0x40);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         // --set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    oled_write_command(0x81);                                                   // --set contrast control register
    oled_write_command(OLED_BRIGHTNESS);                                        //  Set SEG Output Current Brightness

    if(OLED_CROSSWISE == oled_display_dir)
    {
        oled_write_command(0xa1);                                               // --Set SEG/Column Mapping     0xa0左右反置 0xa1正常
        oled_write_command(0xc8);                                               // Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    }
    else
    {
        oled_write_command(0xa0);                                               // --Set SEG/Column Mapping     0xa0左右反置 0xa1正常
        oled_write_command(0xc0);                                               // Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    }

    oled_write_command(0xa6);                                                   // --set normal display
    oled_write_command(0xa8);                                                   // --set multiplex ratio(1 to 64)
    oled_write_command(0x3f);                                                   // --1/64 duty
    oled_write_command(0xd3);                                                   // -set display offset  Shift Mapping RAM Counter (0x00~0x3F)
    oled_write_command(0x00);                                                   // -not offset
    oled_write_command(0xd5);                                                   // --set display clock divide ratio/oscillator frequency
    oled_write_command(0x80);                                                   // --set divide ratio, Set Clock as 100 Frames/Sec
    oled_write_command(0xd9);                                                   // --set pre-charge period
    oled_write_command(0xf1);                                                   // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    oled_write_command(0xda);                                                   // --set com pins hardware configuration
    oled_write_command(0x12);
    oled_write_command(0xdb);                                                   // --set vcomh
    oled_write_command(0x40);                                                   // Set VCOM Deselect Level
    oled_write_command(0x20);                                                   // -Set Page Addressing Mode (0x00/0x01/0x02)
    oled_write_command(0x02);                                                   //
    oled_write_command(0x8d);                                                   // --set Charge Pump enable/disable
    oled_write_command(0x14);                                                   // --set(0x10) disable
    oled_write_command(0xa4);                                                   //  Disable Entire Display On (0xa4/0xa5)
    oled_write_command(0xa6);                                                   //  Disable Inverse Display On (0xa6/a7)
    oled_write_command(0xaf);                                                   // --turn on oled panel
//    OLED_CS(1);

    oled_clear();                                                               // 初始清屏
    oled_set_coordinate(0, 0);
}
