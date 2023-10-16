#include "icm42688.h"
#include <stdio.h>

static uint8_t tx, rx;
static uint8_t tx_buff[14];

static float accSensitivity   = 0.0f;
static float gyroSensitivity  = 0.0f;

#define CALIBRATION_SAMPLES 500 // 校准样本数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM42688 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     icm42688_write_register(ICM42688_PWR_MGMT0, 0x80);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void icm42688_write_register (uint8_t reg, uint8_t data)
{
    ICM42688_CS_LOW;
    tx = reg | ICM42688_SPI_W;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    ICM42688_CS_HIGH;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM42688 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     icm42688_read_register(ICM42688_WHO_AM_I);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t icm42688_read_register (uint8_t reg)
{
    ICM42688_CS_LOW;
    tx = reg | ICM42688_SPI_R;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    tx = 0xff;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    ICM42688_CS_HIGH;
    return rx;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM42688 读数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     icm42688_read_registers(ICM42688_ACCEL_WOM_X_THR, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void icm42688_read_registers (uint8_t reg, uint8_t *data, uint32_t len)
{
    ICM42688_CS_LOW;
    tx = reg | ICM42688_SPI_R;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&hspi2, tx_buff, data, len, 55);
    ICM42688_CS_HIGH;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM42688 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     icm42688_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm42688_self_check (void)
{
    uint8_t dat = 0, return_state = 0;
    uint16_t timeout_count = 0;

    while(0x47 != dat)                                                          // 判断 ID 是否正确
    {
        if(ICM42688_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state =  1;
            break;
        }
        dat = icm42688_read_register(ICM42688_WHO_AM_I);
        HAL_Delay(1);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 ICM42688
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     icm42688Init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm42688Init(void)
{
    uint8_t val = 0x0, return_state = 0;
    uint16_t timeout_count = 0;

    HAL_Delay(10);                                                        // 上电延时

    do
    {
        if(icm42688_self_check())
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 ICM42688 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            printf("icm42688 self check error.\r\n");
            return_state = 1;
            break;
        }

        icm42688_write_register(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
        icm42688_write_register(ICM42688_REG_BANK_SEL, 0x01); //软复位传感器
        HAL_Delay(100);

        icm42688_write_register(ICM42688_REG_BANK_SEL, 1); //设置bank 1区域寄存器
        icm42688_write_register(ICM42688_INTF_CONFIG4, 0x02); //设置为4线SPI通信

        icm42688_write_register(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
        icm42688_write_register(ICM42688_FIFO_CONFIG, 0x40); //Stream-to-FIFO Mode(page63)

        val = icm42688_read_register(ICM42688_INT_SOURCE0);
        icm42688_write_register(ICM42688_INT_SOURCE0, 0x00);
        icm42688_write_register(ICM42688_FIFO_CONFIG2, 0x00); // watermark
        icm42688_write_register(ICM42688_FIFO_CONFIG3, 0x02); // watermark
        icm42688_write_register(ICM42688_INT_SOURCE0, val);
        icm42688_write_register(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

        icm42688_write_register(ICM42688_REG_BANK_SEL, 0x00);
        icm42688_write_register(ICM42688_INT_CONFIG, 0x36);

        icm42688_write_register(ICM42688_REG_BANK_SEL, 0x00);
        val = icm42688_read_register(ICM42688_INT_SOURCE0);
        val |= (1 << 2); //FIFO_THS_INT1_ENABLE
        icm42688_write_register(ICM42688_INT_SOURCE0, val);

        switch(ICM42688_ACC_SAMPLE_DEFAULT)
        {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
            case AFS_2G:
                accSensitivity = 2.0f / 32768.0f;
                break;
            case AFS_4G:
                accSensitivity = 4.0f / 32768.0f;
                break;
            case AFS_8G:
                accSensitivity = 8.0f / 32768.0f;
                break;
            case AFS_16G:
                accSensitivity = 16.0f / 32768.0f;
                break;
        }
        icm42688_write_register(ICM42688_REG_BANK_SEL, 0x00);
        val = icm42688_read_register(ICM42688_ACCEL_CONFIG0);//page74
        val |= (AFS_8G << 5);   //量程 ±8g
        val |= (AODR_50Hz);     //输出速率 50HZ
        icm42688_write_register(ICM42688_ACCEL_CONFIG0, val);

        switch(ICM42688_GYRO_SAMPLE_DEFAULT)
        {
            case GFS_15_125DPS:
                gyroSensitivity = 15.125f / 32768.0f;
                break;
            case GFS_31_25DPS:
                gyroSensitivity = 31.25f / 32768.0f;
                break;
            case GFS_62_5DPS:
                gyroSensitivity = 62.5f / 32768.0f;
                break;
            case GFS_125DPS:
                gyroSensitivity = 125.0f / 32768.0f;
                break;
            case GFS_250DPS:
                gyroSensitivity = 250.0f / 32768.0f;
                break;
            case GFS_500DPS:
                gyroSensitivity = 500.0f / 32768.0f;
                break;
            case GFS_1000DPS:
                gyroSensitivity = 1000.0f / 32768.0f;
                break;
            case GFS_2000DPS:
                gyroSensitivity = 2000.0f / 32768.0f;
                break;
        }
        icm42688_write_register(ICM42688_REG_BANK_SEL, 0x00);
        val = icm42688_read_register(ICM42688_GYRO_CONFIG0);//page73
        val |= (GFS_1000DPS << 5);   //量程 ±1000dps
        val |= (GODR_50Hz);     //输出速率 50HZ
        icm42688_write_register(ICM42688_GYRO_CONFIG0, val);


        icm42688_write_register(ICM42688_REG_BANK_SEL, 0x00);
        val = icm42688_read_register(ICM42688_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值(page72)
        val &= ~(1 << 5);//使能温度测量
        val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        icm42688_write_register(ICM42688_PWR_MGMT0, val);
        HAL_Delay(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

    }while(0);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取ICM42688陀螺仪和加速度计数据
// 参数说明     Axis3i16类型指针
// 返回参数     uint8
// 使用示例     icm42688AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm42688AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw)
{
    uint8_t dat[12];                                             //加速度计,陀螺仪数据

    icm42688_read_registers(ICM42688_ACCEL_DATA_X1, dat, 12);
    _accRaw->x = (int16_t) (((uint16_t)dat[0] << 8 | dat[1]));
    _accRaw->y = (int16_t) -(((uint16_t)dat[2] << 8 | dat[3]));
    _accRaw->z = (int16_t) -(((uint16_t)dat[4] << 8 | dat[5]));

    _gyroRaw->x = (int16_t) (((uint16_t)dat[6] << 8 | dat[7]));
    _gyroRaw->y = (int16_t) -(((uint16_t)dat[8] << 8 | dat[9]));
    _gyroRaw->z = (int16_t) -(((uint16_t)dat[10] << 8 | dat[11]));

    return 1;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM42688 加速度计数据转换为实际物理数据
// 参数说明     Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift
// 返回参数     void
// 备注信息     单位为 (m/s^2)
//-------------------------------------------------------------------------------------------------------------------
void icm42688AccTransformUnit(Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift)
{
    //也许你看不出来我在这翻来翻去的在干啥，其实我没录写代码的素材，所以现在就瞎几把翻翻来水点时长
    _acc_f->x = (float) (_acc->x - _acc_drift->x) * 9.8f * accSensitivity;;
    _acc_f->y = (float) (_acc->y - _acc_drift->y) * 9.8f * accSensitivity;
    _acc_f->z = (float) (_acc->z ) * 9.8f * accSensitivity;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM42688 陀螺仪数据转换为实际物理数据
// 参数说明     Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift
// 返回参数     void
// 备注信息     单位为弧度值
//-------------------------------------------------------------------------------------------------------------------
void icm42688GyroTransformUnit(Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift)
{
    _gyro_f->x = (float) (_gyro->x - _gyro_drift->x) * gyroSensitivity * (3.1415f / 180);
    _gyro_f->y = (float) (_gyro->y - _gyro_drift->y) * gyroSensitivity * (3.1415f / 180);
    _gyro_f->z = (float) (_gyro->z - _gyro_drift->z) * gyroSensitivity * (3.1415f / 180);
}


void ICM42688P_Gyro_And_Acc_Calibrate(Axis3i16* _gyro_drift, Axis3i16* _acc_drift){

    //暂时使用
    Axis3i16 _gyro = {0,0,0};
    Axis3i16 _acc = {0,0,0};

    //为了防止越界，这个数据类型整的大点
    Axis3i64 _sum_gyro = {0,0,0};
    Axis3i64 _sum_acc = {0,0,0};

    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        icm42688AccAndGyroRead(&_acc, &_gyro);

        _sum_gyro.x += _gyro.x;
        _sum_gyro.y += _gyro.y;
        _sum_gyro.z += _gyro.z;

        _sum_acc.x +=_acc.x;
        _sum_acc.y +=_acc.y;
        _sum_acc.z +=_acc.z;

        HAL_Delay(1);
    }
    _gyro_drift->x = (int16_t) (_sum_gyro.x / CALIBRATION_SAMPLES);
    _gyro_drift->y = (int16_t) (_sum_gyro.y / CALIBRATION_SAMPLES);
    _gyro_drift->z = (int16_t) (_sum_gyro.z / CALIBRATION_SAMPLES);

    _acc_drift->x = (int16_t) (_sum_acc.x / CALIBRATION_SAMPLES);
    _acc_drift->y = (int16_t) (_sum_acc.y / CALIBRATION_SAMPLES);
    _acc_drift->z = (int16_t) (_sum_acc.z / CALIBRATION_SAMPLES);
}