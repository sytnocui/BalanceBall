//
// Created by 10798 on 2023/9/2.
//

#ifndef BALANCEBALL_DAMPER_H
#define BALANCEBALL_DAMPER_H

#include <stdint-gcc.h>

typedef struct
{
    float K_damp;

    float in;
    float hold;
    float out;

    float (*update) (void* damper);

} damper_t;

#endif //BALANCEBALL_DAMPER_H
