#ifndef _icm20602_h_
#define _icm20602_h_

#include "main.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "senser_types.h"

#define ICM20602_CS_HIGH HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin,1)
#define ICM20602_CS_LOW  HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin,0)
typedef enum
{
    ICM20602_ACC_SAMPLE_SGN_2G ,                                                // 加速度计量程 ±2G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
    ICM20602_ACC_SAMPLE_SGN_4G ,                                                // 加速度计量程 ±4G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
    ICM20602_ACC_SAMPLE_SGN_8G ,                                                // 加速度计量程 ±8G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
    ICM20602_ACC_SAMPLE_SGN_16G,                                                // 加速度计量程 ±16G (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
}icm20602_acc_sample_config;

typedef enum
{
    ICM20602_GYRO_SAMPLE_SGN_250DPS ,                                           // 陀螺仪量程 ±250DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    ICM20602_GYRO_SAMPLE_SGN_500DPS ,                                           // 陀螺仪量程 ±500DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    ICM20602_GYRO_SAMPLE_SGN_1000DPS,                                           // 陀螺仪量程 ±1000DPS (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    ICM20602_GYRO_SAMPLE_SGN_2000DPS,                                           // 陀螺仪量程 ±2000DPS (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
}icm20602_gyro_sample_config;

#define ICM20602_ACC_SAMPLE_DEFAULT     ( ICM20602_ACC_SAMPLE_SGN_8G )          // 在这设置默认的 加速度计 初始化量程
#define ICM20602_GYRO_SAMPLE_DEFAULT    ( ICM20602_GYRO_SAMPLE_SGN_2000DPS )    // 在这设置默认的 陀螺仪   初始化量程

#define ICM20602_TIMEOUT_COUNT      ( 0x00FF )                                  // ICM20602 超时计数

//================================================定义 ICM20602 内部地址================================================
#define ICM20602_DEV_ADDR           ( 0x69 )                                    // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
#define ICM20602_SPI_W              ( 0x00 )
#define ICM20602_SPI_R              ( 0x80 )

#define ICM20602_XG_OFFS_TC_H       ( 0x04 )
#define ICM20602_XG_OFFS_TC_L       ( 0x05 )
#define ICM20602_YG_OFFS_TC_H       ( 0x07 )
#define ICM20602_YG_OFFS_TC_L       ( 0x08 )
#define ICM20602_ZG_OFFS_TC_H       ( 0x0A )
#define ICM20602_ZG_OFFS_TC_L       ( 0x0B )
#define ICM20602_SELF_TEST_X_ACCEL  ( 0x0D )
#define ICM20602_SELF_TEST_Y_ACCEL  ( 0x0E )
#define ICM20602_SELF_TEST_Z_ACCEL  ( 0x0F )
#define ICM20602_XG_OFFS_USRH       ( 0x13 )
#define ICM20602_XG_OFFS_USRL       ( 0x14 )
#define ICM20602_YG_OFFS_USRH       ( 0x15 )
#define ICM20602_YG_OFFS_USRL       ( 0x16 )
#define ICM20602_ZG_OFFS_USRH       ( 0x17 )
#define ICM20602_ZG_OFFS_USRL       ( 0x18 )
#define ICM20602_SMPLRT_DIV         ( 0x19 )
#define ICM20602_CONFIG             ( 0x1A )
#define ICM20602_GYRO_CONFIG        ( 0x1B )
#define ICM20602_ACCEL_CONFIG       ( 0x1C )
#define ICM20602_ACCEL_CONFIG_2     ( 0x1D )
#define ICM20602_LP_MODE_CFG        ( 0x1E )
#define ICM20602_ACCEL_WOM_X_THR    ( 0x20 )
#define ICM20602_ACCEL_WOM_Y_THR    ( 0x21 )
#define ICM20602_ACCEL_WOM_Z_THR    ( 0x22 )
#define ICM20602_FIFO_EN            ( 0x23 )
#define ICM20602_FSYNC_INT          ( 0x36 )
#define ICM20602_INT_PIN_CFG        ( 0x37 )
#define ICM20602_INT_ENABLE         ( 0x38 )
#define ICM20602_FIFO_WM_INT_STATUS ( 0x39 )
#define ICM20602_INT_STATUS         ( 0x3A )
#define ICM20602_ACCEL_XOUT_H       ( 0x3B )
#define ICM20602_ACCEL_XOUT_L       ( 0x3C )
#define ICM20602_ACCEL_YOUT_H       ( 0x3D )
#define ICM20602_ACCEL_YOUT_L       ( 0x3E )
#define ICM20602_ACCEL_ZOUT_H       ( 0x3F )
#define ICM20602_ACCEL_ZOUT_L       ( 0x40 )
#define ICM20602_TEMP_OUT_H         ( 0x41 )
#define ICM20602_TEMP_OUT_L         ( 0x42 )
#define ICM20602_GYRO_XOUT_H        ( 0x43 )
#define ICM20602_GYRO_XOUT_L        ( 0x44 )
#define ICM20602_GYRO_YOUT_H        ( 0x45 )
#define ICM20602_GYRO_YOUT_L        ( 0x46 )
#define ICM20602_GYRO_ZOUT_H        ( 0x47 )
#define ICM20602_GYRO_ZOUT_L        ( 0x48 )
#define ICM20602_SELF_TEST_X_GYRO   ( 0x50 )
#define ICM20602_SELF_TEST_Y_GYRO   ( 0x51 )
#define ICM20602_SELF_TEST_Z_GYRO   ( 0x52 )
#define ICM20602_FIFO_WM_TH1        ( 0x60 )
#define ICM20602_FIFO_WM_TH2        ( 0x61 )
#define ICM20602_SIGNAL_PATH_RESET  ( 0x68 )
#define ICM20602_ACCEL_INTEL_CTRL   ( 0x69 )
#define ICM20602_USER_CTRL          ( 0x6A )
#define ICM20602_PWR_MGMT_1         ( 0x6B )
#define ICM20602_PWR_MGMT_2         ( 0x6C )
#define ICM20602_I2C_IF             ( 0x70 )
#define ICM20602_FIFO_COUNTH        ( 0x72 )
#define ICM20602_FIFO_COUNTL        ( 0x73 )
#define ICM20602_FIFO_R_W           ( 0x74 )
#define ICM20602_WHO_AM_I           ( 0x75 )
#define ICM20602_XA_OFFSET_H        ( 0x77 )
#define ICM20602_XA_OFFSET_L        ( 0x78 )
#define ICM20602_YA_OFFSET_H        ( 0x7A )
#define ICM20602_YA_OFFSET_L        ( 0x7B )
#define ICM20602_ZA_OFFSET_H        ( 0x7D )
#define ICM20602_ZA_OFFSET_L        ( 0x7E )


//================================================ICM20602 应用层函数==================================================
uint8_t icm20602_self_check (void);
uint8_t icm20602Init(void);
uint8_t icm20602AccAndGyroRead(Axis3i16* _accRaw,Axis3i16* _gyroRaw);
void icm20602AccTransformUnit(Axis3i16* _acc, Axis3f* _acc_f, Axis3i16* _acc_drift);
void icm20602GyroTransformUnit(Axis3i16* _gyro, Axis3f* _gyro_f, Axis3i16* _gyro_drift);
void ICM20602_Gyro_And_Acc_Calibrate(Axis3i16* _gyro_drift, Axis3i16* _acc_drift);

#endif

