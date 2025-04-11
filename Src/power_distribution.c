/**
 * @file power_distribution.c
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 动力分配
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <power_distribution.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "uuv_math.h"
#include "coordinate_conversion.h"

/**
 * @brief 推力曲线模型(  T = t1 * n * |n| - t2 * v * |n| )初始化
 *
 * @param p_tc 推力曲线模型
 * @param t1 系数
 * @param t2 速度衰减系数
 * @param t_max 推力正向上限,取正值
 * @param t_min 推力反向上限,取负值
 * @param s_middle 信号中值
 */
void ThrustCruve_Init(ThrustCruve *p_tc, float t1, float t2, float t_max, float t_min, float s_middle)
{
    if (t1 < 0)
        t1 = -t1;

    if (t2 < 0)
        t2 = -t2;

    p_tc->t1 = t1;
    p_tc->t2 = t2;

    p_tc->t_max    = t_max;
    p_tc->t_min    = t_min;
    p_tc->s_middle = s_middle;
}

/**
 * @brief 通过推力数据获取输出的信号数据
 *
 * @param p_tc 推力曲线模型
 * @param thrust 推力数据
 * @param speed 推进器方向上的速度
 */
float ThrustCruve_GetSignal(ThrustCruve *p_tc, float thrust, float speed)
{
    // 限制最大推力
    if (thrust < p_tc->t_min)
        thrust = p_tc->t_min;
    if (thrust > p_tc->t_max)
        thrust = p_tc->t_max;

    // 判断正反推力
    if (thrust > 0.0001f * p_tc->t_max)
    {
        float delta = p_tc->t2 * p_tc->t2 * speed * speed + 4.0f * p_tc->t1 * thrust;

        return (sqrt(delta) + p_tc->t2 * speed) / 2.0f / p_tc->t1 + p_tc->s_middle;
    }
    else if (thrust < 0.0001f * p_tc->t_min)
    {
        float delta = p_tc->t2 * p_tc->t2 * speed * speed - 4.0f * p_tc->t1 * thrust;

        return (-sqrt(delta) + p_tc->t2 * speed) / 2.0f / p_tc->t1 + p_tc->s_middle;
    }

    return p_tc->s_middle;
}

/**
 * @brief 初始化电机角度
 *
 * @param p_ma 电机角度寄存器地址
 * @param rx
 * @param ry
 * @param rz
 */
void MotorAtt_Init(BaseSys *p_ma, float rx, float ry, float rz)
{
    BaseSys_SetbyParams(p_ma, 0.0f, 0.0f, 0.0f, rx, ry, rz);
}

/**
 * @brief 获取流经电机的水流速度
 *
 * @param p_ma 电机角度寄存器地址
 * @param base_spd_xyz
 * @param fluid_spd_xyz
 * @return float
 */
float MotorAtt_GetSpd(BaseSys *p_ma, float *base_spd_xyz, float *fluid_spd_xyz)
{
    BaseSys *world_spd = (BaseSys *)malloc(sizeof(BaseSys *));
    BaseSys_SetbyParams(world_spd, -base_spd_xyz[0] + fluid_spd_xyz[0], -base_spd_xyz[1] + fluid_spd_xyz[1], -base_spd_xyz[2] + fluid_spd_xyz[2],
                        0.0f, 0.0f, 0.0f);
    BaseSys *intermediate_basesys = (BaseSys *)malloc(sizeof(BaseSys *));
    BaseSys_Init(intermediate_basesys);

    // 计算
    World2Base(p_ma, world_spd, intermediate_basesys);

    float spd_motor_y = intermediate_basesys->vector.y;

    free(intermediate_basesys);
    free(world_spd);

    return spd_motor_y;
}


/**
 * @brief 根据参数初始化推进器对应的推力分配器
 *
 * @param p_ta 推进器对应的推力分配器
 * @param x_cf X轴上分配推力的参数
 * @param y_cf Y轴上分配推力的参数
 * @param z_cf Z轴上分配推力的参数
 * @param rx_cf X轴上分配扭矩的参数
 * @param ry_cf Y轴上分配扭矩的参数
 * @param rz_cf Z轴上分配扭矩的参数
 */
void ThrustAllocate_InitfromParams(ThrustAllocate *p_ta, float x_cf, float y_cf, float z_cf, float rx_cf, float ry_cf, float rz_cf)
{
    p_ta->x_cf  = x_cf;
    p_ta->y_cf  = y_cf;
    p_ta->z_cf  = z_cf;
    p_ta->rx_cf = rx_cf;
    p_ta->ry_cf = ry_cf;
    p_ta->rz_cf = rz_cf;
}


/**
 * @brief 根据数组初始化推进器对应的推力分配器
 *
 * @param p_ta 推进器对应的推力分配器
 * @param params 包含参数信息的数组
 */
void ThrustAllocate_InitfromArray(ThrustAllocate *p_ta, float *params)
{
    p_ta->x_cf  = params[0];
    p_ta->y_cf  = params[1];
    p_ta->z_cf  = params[2];
    p_ta->rx_cf = params[3];
    p_ta->ry_cf = params[4];
    p_ta->rz_cf = params[5];
}


/**
 * @brief 根据输入参数分配扭矩
 *
 * @param p_ta 推进器对应的推力分配器
 * @param x_thrust X轴上的推力
 * @param y_thrust Y轴上的推力
 * @param z_thrust Z轴上的推力
 * @param x_torque X轴上的扭矩
 * @param y_torque Y轴上的扭矩
 * @param z_torque Z轴上的扭矩
 * @return float
 */
float ThrustAllocate_AllocatefromParams(ThrustAllocate *p_ta, float x_thrust, float y_thrust, float z_thrust, float x_torque, float y_torque,
                                        float z_torque)
{
    float thrust = 0;

    thrust += x_thrust * p_ta->x_cf;
    thrust += y_thrust * p_ta->y_cf;
    thrust += z_thrust * p_ta->z_cf;
    thrust += x_torque * p_ta->rx_cf;
    thrust += y_torque * p_ta->ry_cf;
    thrust += z_torque * p_ta->rz_cf;

    return thrust;
}

/**
 * @brief 根据输入的广义力分配扭矩
 *
 * @param p_ta 推进器对应的推力分配器
 * @param t 六维广义力
 * @return float
 */
float ThrustAllocate_AllocatefromVector6d(ThrustAllocate *p_ta, Vector6d *t)
{
    float thrust = 0;

    thrust += t->x * p_ta->x_cf;
    thrust += t->y * p_ta->y_cf;
    thrust += t->z * p_ta->z_cf;
    thrust += t->rx * p_ta->rx_cf;
    thrust += t->ry * p_ta->ry_cf;
    thrust += t->rz * p_ta->rz_cf;

    return thrust;
}
