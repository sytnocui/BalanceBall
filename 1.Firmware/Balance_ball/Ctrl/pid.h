//
// Created by 10798 on 2023/1/3.
//

#ifndef ATK_F405_FW_PID_H
#define ATK_F405_FW_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"

typedef struct
{
    //PID运算模式
    uint8_t mode;
    //PID 三个基本参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //PID最大输出
    float max_iout; //PID最大积分输出

    float set;	  //PID目标值
    float fdb;	  //PID当前值

    float out;		//三项叠加输出
    float Pout;		//比例项输出
    float Iout;		//积分项输出
    float Dout;		//微分项输出
    //微分项最近三个值 0最新 1上一次 2上上次
    float Dbuf[3];
    //误差项最近三个值 0最新 1上一次 2上上次
    float error[3];

} pid_calc_t;


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};


void PID_init(pid_calc_t *pid, uint8_t mode, float kp, float ki, float kd, float max_out, float max_iout);
void PID_clear(pid_calc_t *pid);
float PID_calc(pid_calc_t *pid, float ref, float set);

#ifdef __cplusplus
}
#endif

#endif //ATK_F405_FW_PID_H
