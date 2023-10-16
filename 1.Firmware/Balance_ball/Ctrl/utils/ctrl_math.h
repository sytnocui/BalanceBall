//
// Created by 10798 on 2023/1/10.
//

#ifndef ATK_F405_FW_CTRL_MATH_H
#define ATK_F405_FW_CTRL_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define FAST_MATH


// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define M_LN2f      0.69314718055994530942f
#define M_Ef        2.71828182845904523536f

// angle trans
#define DEG2RAD		0.0174532925f
#define RAD2DEG		57.29578f



#define MIN(a, b) 	(((a) < (b)) ? (a) : (b))
#define MAX(a, b) 	(((a) > (b)) ? (a) : (b))

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo);
float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo);

int constrain(int amt, int low, int high);
float constrainf(float amt, float low, float high);

// 三角计算
float sin_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);

#ifdef __cplusplus
}
#endif
#endif //ATK_F405_FW_CTRL_MATH_H
