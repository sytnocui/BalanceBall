//
// Created by 10798 on 2023/7/25.
//

#include <utils/ctrl_math.h>
#include "ctrl_sin.h"


//三个飞轮的相位
float theta1 = 0.0f;
float theta2 = -M_PIf * 2 / 3;
float theta3 = -M_PIf * 4 / 3;
//周期
float period = 1000.0f;//单位：ms

float gain = 30.0f;
float dcOffset = 30.0f;