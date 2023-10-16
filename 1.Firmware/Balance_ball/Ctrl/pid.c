//
// Created by 10798 on 2023/1/3.
//

#include <stdio.h>
#include <stddef.h>
#include "pid.h"


float LimitMax(float input, float max)
{
    if (input > max)
    {
        input = max;
    }
    else if (input < -max)
    {
        input = -max;
    }
    return input;
}


void PID_init(pid_calc_t *pid, uint8_t mode, float kp, float ki, float kd, float max_out, float max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


void PID_clear(pid_calc_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    //当前误差清零
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    //微分项清零
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    //输出清零
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    //目标值和当前值清零
    pid->fdb = pid->set = 0.0f;
}


float PID_calc(pid_calc_t *pid, float ref, float set)
{
    //判断传入的PID指针不为空
    if (pid == NULL)
    {
        return 0.0f;
    }
    //存放过去两次计算的误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    //设定目标值和当前值到结构体成员
    pid->set = set;
    pid->fdb = ref;
    //计算最新的误差值
    pid->error[0] = set - ref;
    //判断PID设置的模式
    if (pid->mode == PID_POSITION)
    {
        //位置式PID
        //比例项计算输出
        pid->Pout = pid->Kp * pid->error[0];
        //积分项计算输出
        pid->Iout += pid->Ki * pid->error[0];
        //存放过去两次计算的微分误差值
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        //当前误差的微分用本次误差减去上一次误差来计算
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        //微分项输出
        pid->Dout = pid->Kd * pid->Dbuf[0];
        //对积分项进行限幅
        pid->out = LimitMax(pid->Iout, pid->max_iout);
        //叠加三个输出到总输出
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        //对总输出进行限幅
        pid->out = LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        //增量式PID
        //以本次误差与上次误差的差值作为比例项的输入带入计算
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        //以本次误差作为积分项带入计算
        pid->Iout = pid->Ki * pid->error[0];
        //迭代微分项的数组
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        //以本次误差与上次误差的差值减去上次误差与上上次误差的差值作为微分项的输入带入计算
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        //叠加三个项的输出作为总输出
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        //对总输出做一个先限幅
        pid->out = LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}




