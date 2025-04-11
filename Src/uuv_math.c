/**
 * @file uuv_math.c
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 此UuvLib中基于C实现的一些数学求解工具
 * @version 0.1
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "uuv_math.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**
 * @brief 浮点型数值限幅，限制输入量的绝对值
 *
 * @param input 输入值
 * @param limit 限制
 * @return float 限制后结果
 */
float f_limit(float input, float limit)
{
    if (input < -fabs(limit))
        return -fabs(limit);
    else if (input > fabs(limit))
        return fabs(limit);
    else
        return input;
}

/**
 * @brief 给直线两端点求输入x对应的y
 *
 * @param startx 直线起始点X坐标
 * @param endx 直线终止点X坐标
 * @param starty 直线起始点Y坐标
 * @param endy 直线终止点Y坐标
 * @param inputx 输入x坐标
 * @return float 输出y坐标
 */
float line_x2y(float startx, float endx, float starty, float endy, float inputx)
{
    float k = (endy - starty) / (endx - startx);
    return starty + k * (inputx - startx);
}

/**
 * @brief 利用卡尔达诺公式求解单调增三次函数的零点(ax^3+bx^2+cx+d=0)
 *
 * @param a 三次项系数
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return float 所求解，当所求三次函数非单调增时返回0
 */
float solve_cubic_equation(float a, float b, float c, float d)
{
    if (a != 0)
    {
        float p     = c / a - (b * b) / (3 * a * a);
        float q     = (2 * b * b * b) / (27 * a * a * a) - (b * c) / (3 * a * a) + d / a;
        float delta = q * q + 4 * p * p * p / 27;
        float u, v, t1, t2;

        if (delta > 0)
        {
            // 有一个实根和两个共轭复根
            float sqrt_delta = sqrt(delta);
            u                = (-q + sqrt_delta) / 2;
            v                = (-q - sqrt_delta) / 2;

            t1 = cbrt(u);
            t2 = cbrt(v);

            return t1 + t2 - b / (3 * a);
        }
        else
            return 0;
    }
    else // 处理非三次函数
    {
        if (b == 0 && c != 0) // 处理有斜率的一次函数
        {
            return -d / c;
        }
        else // 不处理二次方程 与 水平线
            return 0;
    }
}


/**
 * @brief 角度制下角度值矫正
 *
 * @param angle 待矫正角度值
 * @return float 正确角度值
 */
float angle_correct_deg(float angle)
{
    int cnt = 0;
    if (angle > 180)
    {
        cnt = (int)((angle + 180) / 360);
        angle -= cnt * 360;
    }
    if (angle < -180)
    {
        cnt = (int)((angle - 180) / 360);
        angle -= cnt * 360;
    }
    return angle;
}

/**
 * @brief 弧度制下角度值矫正
 *
 * @param angle 待矫正角度值
 * @return float 正确角度值
 */
float angle_correct_rad(float angle)
{
    int cnt = 0;
    if (angle > pi)
    {
        cnt = (int)((angle + pi) / 2 / pi);
        angle -= cnt * 2 * pi;
    }
    if (angle < -pi)
    {
        cnt = (int)((angle - pi) / 2 / pi);
        angle -= cnt * 2 * pi;
    }
    return angle;
}

/**
 * @brief 两数交换
 *
 * @param a
 * @param b
 */
void swap(float *a, float *b)
{
    float temp = *a;
    *a         = *b;
    *b         = temp;
}

/**
 * @brief 齐次矩阵求逆
 *
 * @param input 输入矩阵
 * @param result 输出的逆矩阵
 */
void homo_mat_inverse(float *input, float *result)
{
    /*
    矩阵的分布为逐列从上至下
    映射关系如下
    0  | 1  | 2  | 3
    4  | 5  | 6  | 7
    8  | 9  | 10 | 11
    12 | 13 | 14 | 15
    */

    // 转置旋转矩阵
    result[0]  = input[0];
    result[5]  = input[5];
    result[10] = input[10];

    result[1] = input[4];
    result[2] = input[8];
    result[6] = input[9];

    result[4] = input[1];
    result[8] = input[2];
    result[9] = input[6];

    // 计算平移
    result[3]  = -(result[0] * input[3] + result[1] * input[7] + result[2] * input[11]);
    result[7]  = -(result[4] * input[3] + result[5] * input[7] + result[6] * input[11]);
    result[11] = -(result[8] * input[3] + result[9] * input[7] + result[10] * input[11]);
}


/**
 * @brief 4*4矩阵求逆
 *
 * @param input 输入矩阵
 * @param result 输出的逆矩阵
 */
void mat_inverse(float *input, float *result)
{
    /*
    矩阵的分布为逐列从上至下
    映射关系如下
    0  | 1  | 2  | 3
    4  | 5  | 6  | 7
    8  | 9  | 10 | 11
    12 | 13 | 14 | 15
    */
    float matrix[4][8];

    // 初始化增广矩阵
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            matrix[i][j]     = input[i * 4 + j];
            matrix[i][j + 4] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 消元过程
    for (int i = 0; i < 4; i++)
    {
        // 如果当前行的对角线元素为0，则需要进行行交换
        if (matrix[i][i] == 0)
        {
            int row = i + 1;
            while (row < 4 && matrix[row][i] == 0)
            {
                row++;
            }

            // 如果找到了非零元素的行，则进行交换
            if (row < 4)
            {
                for (int j = 0; j < 8; j++)
                {
                    swap(&matrix[i][j], &matrix[row][j]);
                }
            }
        }

        // 将对角线元素调整为1
        float scale = matrix[i][i];
        for (int j = 0; j < 8; j++)
        {
            matrix[i][j] /= scale;
        }

        // 消元
        for (int j = 0; j < 4; j++)
        {
            if (j != i)
            {
                float factor = matrix[j][i];
                for (int k = 0; k < 8; k++)
                {
                    matrix[j][k] -= factor * matrix[i][k];
                }
            }
        }
    }

    // 提取逆矩阵部分
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result[i * 4 + j] = matrix[i][j + 4];
        }
    }
}

/**
 * @brief 4*4 矩阵乘法 M1*M2
 *
 * @param m1 矩阵M1
 * @param m2 矩阵M2
 * @param result 矩阵乘法结果
 */
void mat_multiply(float *m1, float *m2, float *result)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result[i * 4 + j] = 0.0;
            for (int k = 0; k < 4; k++)
            {
                result[i * 4 + j] += m1[i * 4 + k] * m2[k * 4 + j];
            }
        }
    }
}


/**
 * @brief 异或校验
 *
 * @param data 数据
 * @param size 数据大小
 * @return uint8_t 校验结果
 */
uint8_t check_data(uint8_t *data, uint8_t size)
{
    uint8_t temp, i;
    temp = data[0];
    for (i = 1; i < size; i++)
    {
        temp = temp ^ data[i];
    }
    return temp;
}
