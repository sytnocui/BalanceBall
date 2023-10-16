//
// Created by 10798 on 2023/7/24.
//

#ifndef _CTRL_TYPES_H
#define _CTRL_TYPES_H



////==================================-base-state-============================================

#include <stdint-gcc.h>

//姿态相关结构体，如姿态角速度，姿态角度，姿态角加速度
typedef struct
{
    uint32_t timestamp;	//时间戳
    float roll;
    float pitch;
    float yaw;
} attitude_t;

//三轴相关结构体，如
struct vec3_s
{
    uint32_t timestamp;
    float x;
    float y;
    float z;
}; //注：这样写，用户没法定义出来 vec3_s 结构体，只能定义出来下面三个结构体

typedef struct vec3_s position_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;        //注意与加速度计测出来的加速度区别，这个不动的时候三轴都是0

////==================================-ctrl-============================================
//模式
typedef enum
{
    modeDisable = 0,
    modeAbs,
    modeVelocity
} ctrl_mode_e;


typedef struct
{
    ctrl_mode_e x;
    ctrl_mode_e y;
    ctrl_mode_e z;
    ctrl_mode_e roll;
    ctrl_mode_e pitch;
    ctrl_mode_e yaw;
} ctrl_mode_t;

//当前飞行状态
typedef struct
{
    attitude_t 	attitude;	    //当前姿态角度
    attitude_t attitudeRate;    //当前姿态角速度
    position_t 	position;	    //当前位置
    velocity_t 	velocity;	    //当前速度
    acc_t acc;				    //当前加速度
} ctrl_state_t;

//目标飞行状态
typedef struct
{
    attitude_t attitude;		//目标姿态角度，单位rad
    attitude_t attitudeRate;	//目标姿态角速度，单位rad/s
    position_t position;        //目标位置，单位m
    velocity_t velocity;      	//目标速度，单位m/s
    ctrl_mode_t ctrl_mode;      //控制模式         在这还挺合适的，之后写面向对象吧
    float thrust;               //油门，单位-1~1    在这还挺合适的，之后写面向对象吧
} ctrl_setpoint_t;

//控制器输出结构体
typedef struct
{    //归一化后的
    float roll;    //roll控制输出   //range -1 ~ 1
    float pitch;   //pitch控制输出  //range -1 ~ 1
    float yaw;     //yaw控制输出    //range -1 ~ 1
    float thrust;  //油门控制输出    //range -1 ~ 1
} ctrl_out_t;

//三轴飞轮速度
typedef struct
{
    float m1;
    float m2;
    float m3;
}motorSpeed_t;



////==================================-ctrl-rc-============================================

#define RC_CHANNLE_ROLL     0    //正
#define RC_CHANNLE_PITCH    1    //反
#define RC_CHANNLE_THRUST   2    //正
#define RC_CHANNLE_YAW      3    //正
#define RC_CHANNLE_ARMED    4
#define RC_CHANNLE_MODE     5

#define RC_MODE_STABILIZED 1000
#define RC_MODE_POSITION 1500
#define RC_MODE_FORWARD 2000

#define RC_ARMED_NO  1000
#define RC_ARMED_YES 2000

//遥控器解析后归一化的结构体
typedef struct
{
    float roll;     //range -1 ~ 1
    float pitch;    //range -1 ~ 1
    float yaw;      //range -1 ~ 1
    float thrust;   //range -1 ~ 1
    uint16_t armed;
    uint16_t mode;
} ctrl_rc_t;


#endif //BALANCEBALL_CTRL_TYPES_H
