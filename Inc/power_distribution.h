/**
 * @file power_distribution.h
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 为潜航器提供的一些关于通信的库
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __THRUST_H
#define __THRUST_H


#pragma pack(push)
#pragma pack(1)


#include "coordinate_conversion.h"


/**
 * @brief   推力曲线\n
 *          推力曲线采用 T = t1 * n * |n| - t2 * v * |n| 拟合。\n
 *          其中 t2 用于描述速度导致的推力衰减,\n
 *          同时T_max为推力正向上限,取正值;T_min为反向推力上限,取负值。
 *          x = x_real - x_middle
 *
 */
typedef struct {
    float t1;       ///< 系数
    float t2;       ///< 速度衰减系数
    float t_max;    ///< 推力正向上限,取正值
    float t_min;    ///< 反向推力上限,取负值
    float s_middle; ///< 信号中值
} ThrustCruve;

void  ThrustCruve_Init(ThrustCruve *p_tc, float t1, float t2, float t_max, float t_min, float s_middle);
float ThrustCruve_GetSignal(ThrustCruve *p_tc, float thrust, float speed);


void  MotorAtt_Init(BaseSys *p_ma, float rx, float ry, float rz);
float MotorAtt_GetSpd(BaseSys *p_ma, float *base_spd_xyz, float *fluid_spd_xyz);


/**
 * @brief   推力分配\n
 *          将推力分配给电机，考虑广义推力必为一六维向量，推力矩阵必定为6列的。
 *
 */
typedef struct {
    float x_cf;  // X轴上分配推力的参数
    float y_cf;  // Y轴上分配推力的参数
    float z_cf;  // Z轴上分配推力的参数
    float rx_cf; // X轴上分配扭矩的参数
    float ry_cf; // Y轴上分配扭矩的参数
    float rz_cf; // Z轴上分配扭矩的参数
} ThrustAllocate;

void  ThrustAllocate_InitfromParams(ThrustAllocate *p_ta, float x_cf, float y_cf, float z_cf, float rx_cf, float ry_cf, float rz_cf);
void  ThrustAllocate_InitfromArray(ThrustAllocate *p_ta, float *params);
float ThrustAllocate_AllocatefromParams(ThrustAllocate *p_ta, float x_thrust, float y_thrust, float z_thrust, float x_torque, float y_torque,
                                        float z_torque);
float ThrustAllocate_AllocatefromVector6d(ThrustAllocate *p_ta, Vector6d *t);


#pragma pack(pop)


#endif