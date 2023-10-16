//
// Created by 10798 on 2023/1/9.
//

#ifndef ATK_F405_FW_ATTITUDE_H
#define ATK_F405_FW_ATTITUDE_H

#include <ctrl_types.h>
#include <senser_types.h>
#include "ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

////---------------------------------陀螺仪相关变量------------------------------
extern Axis3i16 gyro;
extern Axis3i16 acc;
extern Axis3i16 gyro_drift;
extern Axis3i16 acc_drift;
extern Axis3f gyro_f;
extern Axis3f acc_f;
////---------------------------------姿态相关变量------------------------------
extern float q[4];
extern attitude_t state_attitude;
extern attitude_t state_attitude_angle;


void MahonyAHRSupdateIMU(float _q[4], float gx, float gy, float gz, float ax, float ay, float az);
void SytnocuiAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
void AttitudeQuaternionToEulerAngle(const float q[4], attitude_t* attitude);
void AttitudeRadianToAngle(attitude_t* radian, attitude_t* angle);
#ifdef __cplusplus
}
#endif


#endif //ATK_F405_FW_ATTITUDE_H
