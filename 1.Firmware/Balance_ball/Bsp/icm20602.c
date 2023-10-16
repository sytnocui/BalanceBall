#include "icm20602.h"
#include "stdio.h"


float icm20602_transition_factor[2] = {4096, 16.384f};

static uint8_t tx, rx;
static uint8_t tx_buff[14];

#define CALIBRATION_SAMPLES 500 // 校准样本数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM20602 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_write_register (uint8_t reg, uint8_t data)
{
    ICM20602_CS_LOW;
    tx = reg | ICM20602_SPI_W;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    ICM20602_CS_HIGH;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM20602 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     icm20602_read_register(ICM20602_WHO_AM_I);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t icm20602_read_register (uint8_t reg)
{
    ICM20602_CS_LOW;
    tx = reg | ICM20602_SPI_R;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    tx = 0xff;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    ICM20602_CS_HIGH;
    return rx;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM20602 读数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_read_registers (uint8_t reg, uint8_t *data, uint32_t len)
{
    ICM20602_CS_LOW;
    tx = reg | ICM20602_SPI_R;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&hspi2, tx_buff, data, len, 55);
    ICM20602_CS_HIGH;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM20602 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     icm20602_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm20602_self_check (void)
{
    uint8_t dat = 0, return_state = 0;
    uint16_t timeout_count = 0;

    while(0x12 != dat)                                                          // 判断 ID 是否正确
    {
        if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state =  1;
            break;
        }
        dat = icm20602_read_register(ICM20602_WHO_AM_I);
        HAL_Delay(1);
    }
    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取ICM20602陀螺仪和加速度计数据
// 参数说明     Axis3i16类型指针
// 返回参数     uint8
// 使用示例     icm20602AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm20602AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw)
{
    uint8_t dat[14];                                             //加速度计,陀螺仪数据

    icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 14);
    _accRaw->x = (int16_t) (((uint16_t)dat[0] << 8 | dat[1]));
    _accRaw->y = (int16_t) -(((uint16_t)dat[2] << 8 | dat[3]));
    _accRaw->z = (int16_t) -(((uint16_t)dat[4] << 8 | dat[5]));

    _gyroRaw->x = (int16_t) (((uint16_t)dat[8] << 8 | dat[9]));
    _gyroRaw->y = (int16_t) -(((uint16_t)dat[10] << 8 | dat[11]));
    _gyroRaw->z = (int16_t) -(((uint16_t)dat[12] << 8 | dat[13]));

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM20602 加速度计数据转换为实际物理数据
// 参数说明     Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift
// 返回参数     void
// 备注信息     单位为 (m/s^2)
//-------------------------------------------------------------------------------------------------------------------
void icm20602AccTransformUnit(Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift)
{
    _acc_f->x = (float) (_acc->x - _acc_drift->x) * 9.8f / icm20602_transition_factor[0];;
    _acc_f->y = (float) (_acc->y - _acc_drift->y) * 9.8f / icm20602_transition_factor[0];
    _acc_f->z = (float) (_acc->z ) * 9.8f / icm20602_transition_factor[0];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM20602 陀螺仪数据转换为实际物理数据
// 参数说明     Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift
// 返回参数     void
// 备注信息     单位为弧度值
//-------------------------------------------------------------------------------------------------------------------
void icm20602GyroTransformUnit(Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift)
{
    _gyro_f->x = (float) (_gyro->x - _gyro_drift->x) / icm20602_transition_factor[1] * (3.1415f / 180);
    _gyro_f->y = (float) (_gyro->y - _gyro_drift->y) / icm20602_transition_factor[1] * (3.1415f / 180);
    _gyro_f->z = (float) (_gyro->z - _gyro_drift->z) / icm20602_transition_factor[1] * (3.1415f / 180);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 ICM20602
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     icm20602Init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm20602Init(void)
{
    uint8_t val = 0x0, return_state = 0;
    uint16_t timeout_count = 0;

    HAL_Delay(10);                                                        // 上电延时


    do
    {
        if(icm20602_self_check())
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 ICM20602 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            printf("icm20602 self check error.\r\n");
            return_state = 1;
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);                     // 复位设备
        HAL_Delay(2);

        do
        {                                                                       // 等待复位成功
            val = icm20602_read_register(ICM20602_PWR_MGMT_1);
            if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
            {
                // 如果程序在输出了断言信息 并且提示出错位置在这里
                // 那么就是 ICM20602 自检出错并超时退出了
                // 检查一下接线有没有问题 如果没问题可能就是坏了
                printf("icm20602 reset error.\r\n");
                return_state = 1;
                break;
            }
        }while(0x41 != val);
        if(1 == return_state)
        {
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1,     0x01);                 // 时钟设置
        icm20602_write_register(ICM20602_PWR_MGMT_2,     0x00);                 // 开启陀螺仪和加速度计
        icm20602_write_register(ICM20602_CONFIG,         0x01);                 // 176HZ 1KHZ
        icm20602_write_register(ICM20602_SMPLRT_DIV,     0x07);                 // 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)

        // ICM20602_ACCEL_CONFIG 寄存器
        // 设置为 0x00 加速度计量程为 ±2  g   获取到的加速度计数据除以 16384  可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x08 加速度计量程为 ±4  g   获取到的加速度计数据除以 8192   可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x10 加速度计量程为 ±8  g   获取到的加速度计数据除以 4096   可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x18 加速度计量程为 ±16 g   获取到的加速度计数据除以 2048   可以转化为带物理单位的数据 单位 g(m/s^2)
        switch(ICM20602_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                printf("ICM20602_ACC_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_2G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x00);
                icm20602_transition_factor[0] = 16384;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_4G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x08);
                icm20602_transition_factor[0] = 8192;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_8G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x10);
                icm20602_transition_factor[0] = 4096;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_16G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x18);
                icm20602_transition_factor[0] = 2048;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        // ICM20602_GYRO_CONFIG 寄存器
        // 设置为 0x00 陀螺仪量程为 ±250  dps    获取到的陀螺仪数据除以 131     可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x08 陀螺仪量程为 ±500  dps    获取到的陀螺仪数据除以 65.5    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x10 陀螺仪量程为 ±1000 dps    获取到的陀螺仪数据除以 32.8    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x18 陀螺仪量程为 ±2000 dps    获取到的陀螺仪数据除以 16.4    可以转化为带物理单位的数据 单位为 °/s
        switch(ICM20602_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                printf("ICM20602_GYRO_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_250DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x00);
                icm20602_transition_factor[1] = 131.072f;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_500DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x08);
                icm20602_transition_factor[1] = 65.536f;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_1000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x10);
                icm20602_transition_factor[1] = 32.768f;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_2000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x18);
                icm20602_transition_factor[1] = 16.384f;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        icm20602_write_register(ICM20602_ACCEL_CONFIG_2, 0x03);                 // Average 4 samples   44.8HZ   //0x23 Average 16 samples
    }while(0);
    return return_state;
}



void ICM20602_Gyro_And_Acc_Calibrate(Axis3i16* _gyro_drift, Axis3i16* _acc_drift){

    //暂时使用
    Axis3i16 _gyro = {0,0,0};
    Axis3i16 _acc = {0,0,0};

    //为了防止越界，这个数据类型整的大点
    Axis3i64 _sum_gyro = {0,0,0};
    Axis3i64 _sum_acc = {0,0,0};

    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        icm20602AccAndGyroRead(&_acc, &_gyro);

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