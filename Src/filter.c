/**
 * @file filter.c
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 使用C实现的一些滤波器函数
 * @version 0.1
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "filter.h"

/**
 * @brief 卡尔曼滤波器初始化
 *
 * @param p_kf 卡尔曼滤波器
 * @param Q  过程噪声协方差；环境噪声越大，Q越大
 * @param R  测量噪声协方差；测量误差越大，R越大
 */
void KalmanFilter_Init(KalmanFilter *p_kf, float Q, float R)
{
    p_kf->Q = Q;
    p_kf->R = R;

    p_kf->x_est = 0.0; // 估计值
    p_kf->p     = 1.0; // 方差
    p_kf->k     = 0.0; // 卡尔曼增益
}

/**
 * @brief 卡尔曼滤波器更新
 *
 * @param p_kf 卡尔曼滤波器
 * @param measured_value 测量值
 * @return float 校验值
 */
float KalmanFilter_Refersh(KalmanFilter *p_kf, float measured_value)
{
    p_kf->measured_value = measured_value;

    // 预测
    float x_pred = p_kf->x_est;
    float p_pred = p_kf->p + p_kf->Q;

    // 更新
    p_kf->k     = p_pred / (p_pred + p_kf->R);
    p_kf->x_est = x_pred + p_kf->k * (measured_value - x_pred);
    p_kf->p     = (1 - p_kf->k) * p_pred;

    p_kf->checked_value = p_kf->x_est;
    return p_kf->checked_value;
}

/**
 * @brief 均值滤波器初始化
 *
 * @param p_mf 均值滤波器
 * @param length 队列长度
 * @param p_fifo 队列缓冲区起始地址
 */
void MeanFilter_Init(MeanFilter *p_mf, int length, float *p_fifo)
{
    if (length > 0) {
        p_mf->length = length;
    } else {
        p_mf->length = 0;
    }
    p_mf->fifo   = p_fifo;
    p_mf->fifo_p = 0;

    for (int i = 0; i < length; i++) {
        p_mf->fifo[i] = 0;
    }
}

/**
 * @brief 均值滤波器更新
 *
 * @param p_mf 均值滤波器
 * @param measured_value 测量值
 */
float MeanFilter_Refersh(MeanFilter *p_mf, float measured_value)
{
    float sum = 0;

    p_mf->fifo_p++;
    if (p_mf->fifo_p >= p_mf->length) {
        p_mf->fifo_p = 0;
    }
    p_mf->fifo[p_mf->fifo_p] = measured_value;

    for (int i = 0; i < p_mf->length; i++) {
        sum += p_mf->fifo[i];
    }
    p_mf->sum = sum;

    p_mf->mean = sum / p_mf->length;
    return p_mf->mean;
}

/**
 * @brief 联邦卡尔曼滤波器初始化
 *
 * @param p_fkf 联邦卡尔曼滤波器句柄
 * @param Q 过程噪声协方差；环境噪声越大，Q越
 * @param R 测量噪声协方差；测量误差越大，R越
 * @param num 局部滤波器数量
 * @param p_lkf 局部滤波器
 * @param p_weights 局部滤波器权重系数
 */
FederatedKalmanFilter_Init(FederatedKalmanFilter *p_fkf, float Q, float R, int num, KalmanFilter *p_lkf, float *p_weights)
{

    // 初始化主滤波器
    KalmanFilter_Init(&(p_fkf->master), Q, R);

    // 挂载局部滤波器
    if (num < 2) {
        num = 2;
    }
    p_fkf->local_filter_num = num;
    p_fkf->p_local_filters  = p_lkf;
    p_fkf->p_weights        = p_weights;

    // 计算总权重系数
    p_fkf->weight_sum = 0.0f;
    for (int i = 0; i < p_fkf->local_filter_num; i++) {
        p_fkf->weight_sum += p_fkf->p_weights[i];
    }
}

/**
 * @brief Construct a new FederatedKalmanFilter_Update object
 *
 * @param p_fkf 联邦卡尔曼滤波器句柄
 * @param input_buf 输入变量
 */
FederatedKalmanFilter_Update(FederatedKalmanFilter *p_fkf, float *input_buf)
{

    // 更新从滤波器与权重分配
    float checked_value = 0.0f;
    for (int i = 0; i < p_fkf->local_filter_num; i++) {
        checked_value += p_fkf->p_weights[i] * KalmanFilter_Update(&(p_fkf->p_local_filters[i]), input_buf[i]);
    }
    checked_value /= p_fkf->weight_sum;

    // 更新主滤波器
    return KalmanFilter_Update(&(p_fkf->master), checked_value);
}