/**
 * @file coordinate_conversion.c
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 矢量在各坐标系中的变换
 * @version 0.1
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "coordinate_conversion.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "uuv_math.h"

/**
 * @brief 广义坐标初始化
 *
 * @param p_base_sys 完整的广义坐标的指针
 */
void BaseSys_Init(BaseSys *p_base_sys)
{
    p_base_sys->vector.x  = 0;
    p_base_sys->vector.y  = 0;
    p_base_sys->vector.z  = 0;
    p_base_sys->vector.rx = 0;
    p_base_sys->vector.ry = 0;
    p_base_sys->vector.rz = 0;

    vector2matrix(p_base_sys);
}

/**
 * @brief 将 六维向量 导出为 齐次矩阵
 *
 * @param p_base_sys 完整的广义坐标的指针
 */
void vector2matrix(BaseSys *p_base_sys)
{
    float Sx = sin(p_base_sys->vector.rx * deg2rad);
    float Cx = cos(p_base_sys->vector.rx * deg2rad);
    float Sy = sin(p_base_sys->vector.ry * deg2rad);
    float Cy = cos(p_base_sys->vector.ry * deg2rad);
    float Sz = sin(p_base_sys->vector.rz * deg2rad);
    float Cz = cos(p_base_sys->vector.rz * deg2rad);

    // 矩阵计算
    p_base_sys->h_matrix[0] = Cy * Cz;
    p_base_sys->h_matrix[1] = Sx * Sy * Cz - Cx * Sz;
    p_base_sys->h_matrix[2] = Cx * Sy * Cz + Sx * Sz;
    p_base_sys->h_matrix[3] = p_base_sys->vector.x;

    p_base_sys->h_matrix[4] = Cy * Sz;
    p_base_sys->h_matrix[5] = Sx * Sy * Sz + Cx * Cz;
    p_base_sys->h_matrix[6] = Cx * Sy * Sz - Sx * Cz;
    p_base_sys->h_matrix[7] = p_base_sys->vector.y;

    p_base_sys->h_matrix[8]  = -Sy;
    p_base_sys->h_matrix[9]  = Sx * Cy;
    p_base_sys->h_matrix[10] = Cx * Cy;
    p_base_sys->h_matrix[11] = p_base_sys->vector.z;

    p_base_sys->h_matrix[12] = 0.0f;
    p_base_sys->h_matrix[13] = 0.0f;
    p_base_sys->h_matrix[14] = 0.0f;
    p_base_sys->h_matrix[15] = 1.0f;
}

/**
 * @brief 齐次矩阵 导出为  六维向量
 *
 * @param p_base_sys 完整的广义坐标的指针
 */
void matrix2vector(BaseSys *p_base_sys)
{
    p_base_sys->vector.x = p_base_sys->h_matrix[3];
    p_base_sys->vector.y = p_base_sys->h_matrix[7];
    p_base_sys->vector.z = p_base_sys->h_matrix[11];

    float rx1, rx2;
    float ry1, ry2;

    if (p_base_sys->h_matrix[8] != 1.0f || p_base_sys->h_matrix[8] != -1.0f) // 若越界，放弃更新
    {
        ry1 = -asin(p_base_sys->h_matrix[8]);
        ry2 = pi - ry1;

        rx1 = atan2(p_base_sys->h_matrix[9] / cos(ry1), p_base_sys->h_matrix[10] / cos(ry1));
        rx2 = atan2(p_base_sys->h_matrix[9] / cos(ry2), p_base_sys->h_matrix[10] / cos(ry2));

        if (ry1 < pi / 2 && ry1 > -pi / 2)
            p_base_sys->vector.ry = ry1 * rad2deg;
        if (ry2 < pi / 2 && ry2 > -pi / 2)
            p_base_sys->vector.ry = ry2 * rad2deg;

        if (rx1 < pi / 2 && rx1 > -pi / 2)
            p_base_sys->vector.rx = rx1 * rad2deg;
        if (rx2 < pi / 2 && rx2 > -pi / 2)
            p_base_sys->vector.rx = rx2 * rad2deg;

        // if (rz1 < pi && rz1 > -pi)
        p_base_sys->vector.rz =
            atan2(p_base_sys->h_matrix[4] / cos(p_base_sys->vector.ry * deg2rad), p_base_sys->h_matrix[0] / cos(p_base_sys->vector.ry * deg2rad))
            * rad2deg;
        // if (rz2 < pi && rz2 > -pi)
        //     p_full_vector->vector.rz = rz2 * rad2deg;
    }
}

/**
 * @brief 通过参数直接为广义坐标系统赋值
 *
 * @param p_bs 赋值目标
 * @param x
 * @param y
 * @param z
 * @param rx
 * @param ry
 * @param rz
 */
void BaseSys_SetbyParams(BaseSys *p_bs, float x, float y, float z, float rx, float ry, float rz)
{
    p_bs->vector.x  = x;
    p_bs->vector.y  = y;
    p_bs->vector.z  = z;
    p_bs->vector.rx = rx;
    p_bs->vector.ry = ry;
    p_bs->vector.rz = rz;

    vector2matrix(p_bs);
}

/**
 * @brief 通过数组为广义坐标系统赋值
 *
 * @param p_bs 赋值目标
 * @param arr 数组，按顺序为 x,y,z,rx,ry,rz
 */
void BaseSys_SetbyArray(BaseSys *p_bs, float *arr)
{
    memcpy(&(p_bs->vector), arr, sizeof(Vector6d));

    vector2matrix(p_bs);
}

/**
 * @brief 通过六维向量为广义坐标系统赋值
 *
 * @param p_bs 赋值目标
 * @param p_vector
 */
void BaseSys_SetbyVector6d(BaseSys *p_bs, Vector6d *p_vector)
{
    memcpy(&(p_bs->vector), p_vector, sizeof(Vector6d));

    vector2matrix(p_bs);
}

/**
 * @brief 通过广义坐标系统为广义坐标系统赋值
 *
 * @param p_bs_dst 赋值目标
 * @param p_bs_src 赋值来源
 */
void BaseSys_SetbyBaseSys(BaseSys *p_bs_dst, BaseSys *p_bs_src)
{
    memcpy(p_bs_dst, p_bs_src, sizeof(BaseSys));
}

/**
 * @brief 六维向量相加 A1 + A2 = Aout
 *
 * @param p_bs1 六维向量 A1
 * @param p_bs2 六维向量 A2
 * @param p_bs_out 六维向量 Aout
 */
void BaseSysAdd(BaseSys *p_bs1, BaseSys *p_bs2, BaseSys *p_bs_out)
{
    // 齐次矩阵相乘
    mat_multiply((float *)&(p_bs1->h_matrix), (float *)&(p_bs2->h_matrix), (float *)&(p_bs_out->h_matrix));

    // 完成齐次矩阵向六维向量的转换
    matrix2vector(p_bs_out);
}

/**
 * @brief 六维向量相加 A1 - A2 = Aout
 *
 * @param p_bs1 六维向量 A1
 * @param p_bs2 六维向量 A2
 * @param p_bs_out 六维向量 Aout
 */
void BaseSysMinus(BaseSys *p_bs1, BaseSys *p_bs2, BaseSys *p_bs_out)
{
    // 求六维向量A2的逆
    float mat[16] = {0};
    homo_mat_inverse((float *)&(p_bs2->h_matrix), (float *)mat);

    // 齐次矩阵相乘
    mat_multiply((float *)&(p_bs1->h_matrix), (float *)mat, (float *)&(p_bs_out->h_matrix));

    // 完成齐次矩阵向六维向量的转换
    matrix2vector(p_bs_out);
}

/**
 * @brief 坐标系转换 世界坐标转换至其他参考系
 *
 * @param p_bs_base 目标坐标系的基底广义坐标 A1
 * @param p_bs_in_world 世界坐标系中待转换的六维向量 A2
 * @param p_bs_out A2 在 A1 中的广义坐标
 */
void World2Base(BaseSys *p_bs_base, BaseSys *p_bs_in_world, BaseSys *p_bs_out)
{
    // 求六维向量A1的逆
    float mat[16] = {0};
    homo_mat_inverse((float *)&(p_bs_base->h_matrix), (float *)mat);

    // 齐次矩阵相乘
    mat_multiply((float *)mat, (float *)&(p_bs_in_world->h_matrix), (float *)&(p_bs_out->h_matrix));

    // 完成齐次矩阵向六维向量的转换
    matrix2vector(p_bs_out);
}

/**
 * @brief 坐标系转换 其他参考系转换至世界坐标
 *
 * @param p_bs_base 基底坐标系的基底广义坐标 A1
 * @param p_bs_in_base 基底坐标系中待转换的六维向量 A2
 * @param p_bs_out A2 在 世界 中的广义坐标
 */
void Base2World(BaseSys *p_bs_base, BaseSys *p_bs_in_base, BaseSys *p_bs_out)
{
    // 齐次矩阵相乘
    mat_multiply((float *)&(p_bs_base->h_matrix), (float *)&(p_bs_in_base->h_matrix), (float *)&(p_bs_out->h_matrix));

    // 完成齐次矩阵向六维向量的转换
    matrix2vector(p_bs_out);
}

/**
 * @brief 两个坐标之间的矢量变换 坐标系1 至 坐标系2
 *
 * @param p_bs_base_1 坐标系1
 * @param p_bs_base_2 坐标系2
 * @param p_bs_in_base_1 坐标系1中的一个矢量
 * @param p_bs_out 输出矢量
 */
void Base2Base(BaseSys *p_bs_base_1, BaseSys *p_bs_base_2, BaseSys *p_bs_in_base_1, BaseSys *p_bs_out)
{
    BaseSys *intermediate_basesys = (BaseSys *)malloc(sizeof(BaseSys *));
    BaseSys_Init(intermediate_basesys);

    Base2World(p_bs_base_1, p_bs_in_base_1, intermediate_basesys);

    World2Base(p_bs_base_2, intermediate_basesys, p_bs_out);

    free(intermediate_basesys);
}
