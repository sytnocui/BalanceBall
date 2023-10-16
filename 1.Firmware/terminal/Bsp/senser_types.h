//
// Created by 10798 on 2023/7/24.
//

#ifndef BALANCEBALL_SENSER_TYPES_H
#define BALANCEBALL_SENSER_TYPES_H

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}Axis3i16;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
}Axis3i32;

typedef struct
{
    int64_t x;
    int64_t y;
    int64_t z;
}Axis3i64;

typedef struct
{
    float x;
    float y;
    float z;
}Axis3f;

#endif //BALANCEBALL_SENSER_TYPES_H
