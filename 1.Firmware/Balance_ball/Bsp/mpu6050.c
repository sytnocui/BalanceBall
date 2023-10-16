#include <stdio.h>
#include "mpu6050.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPU6050 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void mpu6050_write_register (uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, (MPU6050_DEV_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xfff);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPU6050 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     mpu6050_read_register(MPU6050_SMPLRT_DIV)
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_read_register (uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c2, (MPU6050_DEV_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 0xfff);
    return res;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPU6050 读数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 14);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void mpu6050_read_registers (uint8_t reg, uint8_t *data, uint32_t len)
{
    HAL_I2C_Mem_Read(&hi2c2, (MPU6050_DEV_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, 0xfff);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPU6050 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     if(mpu6050_self1_check())
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_self1_check (void)
{
    uint8_t dat = 0, return_state = 0;
    uint16_t timeout_count = 0;

    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);                           // 解除休眠状态
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);                           // 125HZ采样率
    while(0x07 != dat)
    {
        if(timeout_count ++ > MPU6050_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
        dat = mpu6050_read_register(MPU6050_SMPLRT_DIV);
        HAL_Delay(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取MPU6050陀螺仪和加速度计数据
// 参数说明     Axis3i16类型指针
// 返回参数     uint8
// 使用示例     mpu6050AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw)
{
    uint8_t dat[14];                                             //加速度计,陀螺仪数据

    mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 14);
    _accRaw->x = (int16_t) -(((uint16_t)dat[0] << 8 | dat[1]));
    _accRaw->y = (int16_t) -(((uint16_t)dat[2] << 8 | dat[3]));
    _accRaw->z = (int16_t)(((uint16_t)dat[4] << 8 | dat[5]));

    _gyroRaw->x = (int16_t) -(((uint16_t)dat[8] << 8 | dat[9]));
    _gyroRaw->y = (int16_t) -(((uint16_t)dat[10] << 8 | dat[11]));
    _gyroRaw->z = (int16_t)(((uint16_t)dat[12] << 8 | dat[13]));

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 mpu6050 加速度计数据转换为实际物理数据
// 参数说明     Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift
// 返回参数     void
// 备注信息     单位为 (m/s^2)
//-------------------------------------------------------------------------------------------------------------------
void mpu6050AccTransformUnit (Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift)
{
    switch(MPU6050_ACC_SAMPLE)
    {
        case 0x00:
            _acc_f->x = (float) (_acc->x - _acc_drift->x) * 9.8f / 16384;
            _acc_f->y = (float) (_acc->y - _acc_drift->y) * 9.8f / 16384;
            _acc_f->z = (float) (_acc->z ) * 9.8f / 16384;
            break;
        // 0x00 加速度计量程为:±2g     获取到的加速度计数据 除以 16384      可以转化为带物理单位的数据，单位：(m/s^2)
        case 0x08:
            _acc_f->x = (float) (_acc->x - _acc_drift->x) * 9.8f / 8192;
            _acc_f->y = (float) (_acc->y - _acc_drift->y) * 9.8f / 8192;
            _acc_f->z = (float) (_acc->z ) * 9.8f / 8192;
            break;
        // 0x08 加速度计量程为:±4g     获取到的加速度计数据 除以 8192       可以转化为带物理单位的数据，单位：(m/s^2)
        case 0x10:
            _acc_f->x = (float) (_acc->x - _acc_drift->x) * 9.8f / 4096;
            _acc_f->y = (float) (_acc->y - _acc_drift->y) * 9.8f / 4096;
            _acc_f->z = (float) (_acc->z ) * 9.8f / 4096;
            break;
        // 0x10 加速度计量程为:±8g     获取到的加速度计数据 除以 4096       可以转化为带物理单位的数据，单位：(m/s^2)
        case 0x18:
            _acc_f->x = (float) (_acc->x - _acc_drift->x) * 9.8f / 2048;
            _acc_f->y = (float) (_acc->y - _acc_drift->y) * 9.8f / 2048;
            _acc_f->z = (float) (_acc->z ) * 9.8f / 2048;
            break;
        // 0x18 加速度计量程为:±16g    获取到的加速度计数据 除以 2048       可以转化为带物理单位的数据，单位：(m/s^2)
        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM20602 陀螺仪数据转换为实际物理数据
// 参数说明     Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift
// 返回参数     void
// 备注信息     单位为弧度值
//-------------------------------------------------------------------------------------------------------------------
void mpu6050GyroTransformUnit(Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift)
{
    switch(MPU6050_GYR_SAMPLE)
    {
        case 0x00:
            _gyro_f->x = (float) (_gyro->x - _gyro_drift->x) / 131.0f * (3.1415f / 180);
            _gyro_f->y = (float) (_gyro->y - _gyro_drift->y) / 131.0f * (3.1415f / 180);
            _gyro_f->z = (float) (_gyro->z - _gyro_drift->z) / 131.0f * (3.1415f / 180);
            break;
        // 0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以 131           可以转化为带物理单位的数据，单位为：/s
        case 0x08:
            _gyro_f->x = (float) (_gyro->x - _gyro_drift->x) / 65.5f * (3.1415f / 180);
            _gyro_f->y = (float) (_gyro->y - _gyro_drift->y) / 65.5f * (3.1415f / 180);
            _gyro_f->z = (float) (_gyro->z - _gyro_drift->z) / 65.5f * (3.1415f / 180);
            break;
        // 0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以 65.5          可以转化为带物理单位的数据，单位为：/s
        case 0x10:
            _gyro_f->x = (float) (_gyro->x - _gyro_drift->x) / 32.8f * (3.1415f / 180);
            _gyro_f->y = (float) (_gyro->y - _gyro_drift->y) / 32.8f * (3.1415f / 180);
            _gyro_f->z = (float) (_gyro->z - _gyro_drift->z) / 32.8f * (3.1415f / 180);
            break;
        // 0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以 32.8          可以转化为带物理单位的数据，单位为：/s
        case 0x18:
            _gyro_f->x = (float) (_gyro->x - _gyro_drift->x) / 16.4f * (3.1415f / 180);
            _gyro_f->y = (float) (_gyro->y - _gyro_drift->y) / 16.4f * (3.1415f / 180);
            _gyro_f->z = (float) (_gyro->z - _gyro_drift->z) / 16.4f * (3.1415f / 180);
            break;
        // 0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以 16.4          可以转化为带物理单位的数据，单位为：/s
        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 MPU6050
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     mpu6050Init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050Init(void)
{
    uint8_t return_state = 0;
    HAL_Delay(100);                                                       // 上电延时

    do
    {
        if(mpu6050_self1_check())
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 MPU6050 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            printf("MPU6050 self check error.\r\n");
            return_state = 1;
            break;
        }
        mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);                       // 解除休眠状态
        mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);                       // 125HZ采样率
        mpu6050_write_register(MPU6050_CONFIG, 0x04);

        mpu6050_write_register(MPU6050_GYRO_CONFIG, MPU6050_GYR_SAMPLE);        // 2000
        // GYRO_CONFIG寄存器
        // 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131.2         可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.6          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

        mpu6050_write_register(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE);       // 8g
        // ACCEL_CONFIG寄存器
        // 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

        mpu6050_write_register(MPU6050_USER_CONTROL, 0x00);
        mpu6050_write_register(MPU6050_INT_PIN_CFG, 0x02);
    }while(0);
    return return_state;
}