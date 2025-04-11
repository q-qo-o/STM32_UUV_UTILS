/**
 * @file coordinate_conversion.h
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 矢量在各坐标系中的变换\n
 *          此处仅考虑机器人的位置、姿态变换关系
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __COORDIBATE_CONVERSION_H
#define __COORDIBATE_CONVERSION_H

#pragma pack(push)
#pragma pack(1)


/**
 * @brief 一个结构体，机器人的广义坐标
 *
 *
 */
typedef struct {
    float x;  ///< X 坐标
    float y;  ///< Y 坐标
    float z;  ///< Z 坐标
    float rx; ///< 绕X的旋转角 小角度下近似为 pitch
    float ry; ///< 绕Y的旋转角 小角度下近似为 roll
    float rz; ///< 绕Z的旋转角 小角度下近似为 yaw
} Vector6d;

// 广义坐标系统，包含六维广义坐标和对应的齐次矩阵

/**
 * @brief   广义坐标系统，包含六维广义坐标和对应的齐次矩阵\n
 *          其中齐次矩阵内容大致如下：\n
 *          0|1|2|3\n
 *          4|5|6|7\n
 *          8|9|10|11\n
 *          12|13|14|15
 *
 */
typedef struct {
    Vector6d vector;       ///< 机器人的广义坐标
    float    h_matrix[16]; ///< 齐次矩阵
} BaseSys;

void vector2matrix(BaseSys *p_base_sys);
void matrix2vector(BaseSys *p_base_sys);
void BaseSys_Init(BaseSys *p_base_sys);
void BaseSys_SetbyParams(BaseSys *p_bs, float x, float y, float z, float rx, float ry, float rz);
void BaseSys_SetbyArray(BaseSys *p_bs, float *arr);
void BaseSys_SetbyVector6d(BaseSys *p_bs, Vector6d *p_vector);
void BaseSys_SetbyBaseSys(BaseSys *p_bs, BaseSys *p_bs_src);

// 广义坐标相加减
void BaseSysAdd(BaseSys *p_bs1, BaseSys *p_bs2, BaseSys *p_bs_out);
void BaseSysMinus(BaseSys *p_bs1, BaseSys *p_bs2, BaseSys *p_bs_out);

// 坐标转换系统
void World2Base(BaseSys *p_bs_base, BaseSys *p_bs_in_world, BaseSys *p_bs_out);
void Base2World(BaseSys *p_bs_base, BaseSys *p_bs_in_base, BaseSys *p_bs_out);
void Base2Base(BaseSys *p_bs_base_1, BaseSys *p_bs_base_2, BaseSys *p_bs_in_base_1, BaseSys *p_bs_out);


#pragma pack(pop)


#endif // !__coordinate_conversion