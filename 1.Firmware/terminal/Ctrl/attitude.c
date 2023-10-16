//
// Created by 10798 on 2023/1/9.
//

#include "attitude.h"

#include "utils/bsp_math.h"
#include "ctrl_types.h"
#include "senser_types.h"
#include <math.h>
#include "mpu6050.h"

////---------------------------------陀螺仪相关变量------------------------------
Axis3i16 gyro = {0,0,0};
Axis3i16 acc = {0,0,0};
Axis3i16 gyro_drift = {0,0,0};
Axis3i16 acc_drift = {0,0,0};
Axis3f acc_f = {0,0,0};
Axis3f gyro_f = {0,0,0};
////---------------------------------姿态相关变量--------------------------------
//姿态四元数
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
//欧拉角
attitude_t state_attitude = {0,0.0f,0.0f,0.0f};
//角度制的欧拉角
attitude_t state_attitude_angle = {0,0.0f,0.0f,0.0f};
////--------------------------------------------------------------------------

////--------------------------------------------姿态解算
// Definitions

#define sampleFreq	100.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain     (2.0f * 0.5f)
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

void MahonyAHRSupdateIMU(float _q[4], float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
        recipNorm = 1.0f/sqrtf(ax * ax + ay * ay + az * az); //该函数返回平方根的倒数
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        // 通过四元数得到理论重力加速度向量g
        // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
        // 这样处理目的是减少乘法运算量
        halfvx = _q[1] * _q[3] - _q[0] * _q[2];
        halfvy = _q[0] * _q[1] + _q[2] * _q[3];
        halfvz = _q[0] * _q[0] - 0.5f + _q[3] * _q[3];

        // Error is sum of cross product between estimated and measured direction of gravity
        // 对实际重力加速度向量v与理论重力加速度向量g做外积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
        if(twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);

            // apply integral feedback
            // 应用误差补偿中的积分项
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // 应用误差补偿中的比例项
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 微分方程迭代求解
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = _q[0];
    qb = _q[1];
    qc = _q[2];
    _q[0] += (-qb * gx - qc * gy - _q[3] * gz);
    _q[1] += (qa * gx + qc * gz - _q[3] * gy);
    _q[2] += (qa * gy - qb * gz + _q[3] * gx);
    _q[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
    recipNorm = 1.0f/sqrtf(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm;
    _q[1] *= recipNorm;
    _q[2] *= recipNorm;
    _q[3] *= recipNorm;

    //Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
}


void Gyro_And_Acc_Calibrate(Axis3i16* _gyro_drift, Axis3i16* _acc_drift){

    #define CALIBRATION_SAMPLES 500 // 校准样本数

    //暂时使用
    Axis3i16 _gyro = {0,0,0};
    Axis3i16 _acc = {0,0,0};

    //为了防止越界，这个数据类型整的大点
    Axis3i64 _sum_gyro = {0,0,0};
    Axis3i64 _sum_acc = {0,0,0};

    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        //NOTE:这里换做自己的读取代码
//        icm20602AccAndGyroRead(&_acc, &_gyro);
        mpu6050AccAndGyroRead(&_acc, &_gyro);
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


void AttitudeQuaternionToEulerAngle(const float _q[4], attitude_t* _attitude){
    // 传入的结果是指针，结果返回到传入参数地址内
    // 四元数反算欧拉角
    // 使用子蕤那嫖的正点原子的估算，asinf没有换，正点原子也没有换
    // 自己写的不知道为什么就是不好使，chatgpt一下场，生成的立马好使了
    float w = _q[0], x = _q[1], y = _q[2], z = _q[3];
    // roll
    float sinr_cosp = 2 * (w*x + y*z);
    float cosr_cosp = 1 - 2 * (x*x + y*y);
    _attitude->roll = atan2_approx(sinr_cosp, cosr_cosp);
    // pitch
    float sinp = 2 * (w*y - z*x);
    if (fabsf(sinp) >= 1)
        _attitude->pitch = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _attitude->pitch = asinf(sinp);
    // yaw
    float siny_cosp = 2 * (w*z + x*y);
    float cosy_cosp = 1 - 2 * (y*y + z*z);
    _attitude->yaw = atan2_approx(siny_cosp, cosy_cosp);
}


void AttitudeRadianToAngle(attitude_t* radian, attitude_t* angle){
    angle->roll = radian->roll * 57.29578f;
    angle->pitch = radian->pitch * 57.29578f;
    angle->yaw = radian->yaw * 57.29578f;
}
