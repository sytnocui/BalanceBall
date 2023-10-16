//
// Created by 10798 on 2023/7/23.
//

#ifndef BALANCEBALL_INTERFACE_CAN_H
#define BALANCEBALL_INTERFACE_CAN_H

#include "stdio.h"
#include "common_inc.h"
#include "can.h"
#include "ctrl_types.h"


#define M0 0x001 //偏航轴飞轮
#define M1 0x002 //滚转轴飞轮
#define M2 0x003 //俯仰轴飞轮

typedef union{
    char data_ch[4];
    float data_f;
}driver_data;


void Drive_Init(uint32_t CAN_ID);
void Driver_control(uint32_t CAN_ID, float target);
void Drive_Clear_Error(uint32_t CAN_ID);
void DriverCmdSend(ctrl_rc_t* _rc, motorSpeed_t * _motor);


#endif //BALANCEBALL_INTERFACE_CAN_H
