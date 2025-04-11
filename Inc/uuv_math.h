/**
 * @file uuv_math.h
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 此UuvLib中基于C实现的一些数学求解工具
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __UUV_MATH_H
#define __UUV_MATH_H


#pragma pack(push)
#pragma pack(1)


#define deg2rad 0.01745329252f // 角度制转弧度制
#define rad2deg 57.29577951f   // 弧度制转角度制
#define pi 3.141592654f        // 圆周率

#include <stdint.h>

float   f_limit(float input, float limit);
float   line_x2y(float startx, float endx, float starty, float endy, float inputx);
float   solve_cubic_equation(float a, float b, float c, float d);
float   angle_correct_deg(float angle);
float   angle_correct_rad(float angle);
void    swap(float *a, float *b);
void    homo_mat_inverse(float *input, float *result);
void    mat_inverse(float *input, float *result);
void    mat_multiply(float *m1, float *m2, float *result);
uint8_t check_data(uint8_t *data, uint8_t size);


#pragma pack(pop)


#endif // !__UUV_MATH_H
