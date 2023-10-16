#ifndef _mpu6050_h_
#define _mpu6050_h_

#include "senser_types.h"
#include <i2c.h>

#define MPU6050_TIMEOUT_COUNT       (0x00FF)                                    // MPU6050 超时计数

//================================================定义 MPU6050 内部地址================================================
#define MPU6050_DEV_ADDR            (0xD0>>1)                                   // IIC写入时的地址字节数据，+1为读取

#define MPU6050_SMPLRT_DIV          (0x19)                                      // 陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6050_CONFIG              (0x1A)                                      // 低通滤波频率，典型值：0x06(5Hz)
#define MPU6050_GYRO_CONFIG         (0x1B)                                      // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU6050_ACCEL_CONFIG        (0x1C)                                      // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPU6050_INT_PIN_CFG         (0x37)                                      // 设置6050辅助I2C为直通模式寄存器
#define MPU6050_ACCEL_XOUT_H        (0x3B)
#define MPU6050_ACCEL_XOUT_L        (0x3C)
#define MPU6050_ACCEL_YOUT_H        (0x3D)
#define MPU6050_ACCEL_YOUT_L        (0x3E)
#define MPU6050_ACCEL_ZOUT_H        (0x3F)
#define MPU6050_ACCEL_ZOUT_L        (0x40)
#define MPU6050_GYRO_XOUT_H         (0x43)
#define MPU6050_GYRO_XOUT_L         (0x44)
#define MPU6050_GYRO_YOUT_H         (0x45)
#define MPU6050_GYRO_YOUT_L         (0x46)
#define MPU6050_GYRO_ZOUT_H         (0x47)
#define MPU6050_GYRO_ZOUT_L         (0x48)
#define MPU6050_USER_CONTROL        (0x6A)                                      // 关闭6050对辅助I2C设备的控制
#define MPU6050_PWR_MGMT_1          (0x6B)                                      // 电源管理，典型值：0x00(正常启用)
#define MPU6050_WHO_AM_I            (0x75)                                      // IIC地址寄存器(默认数值0x68，只读)


// FIXME:实测设置这两个数值没有作用，MPU6050只按两个都是0x10来返回数据

#define MPU6050_ACC_SAMPLE          (0x10)                                      // 加速度计量程
// 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
#define MPU6050_GYR_SAMPLE          (0x10)                                      // 陀螺仪量程
// 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131.2         可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.6          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

//================================================定义 MPU6050 内部地址================================================

uint8_t mpu6050AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw);
void mpu6050AccTransformUnit (Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift);
void mpu6050GyroTransformUnit(Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift);
uint8_t mpu6050Init(void);

#endif