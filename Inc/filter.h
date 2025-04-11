/**
 * @file filter.h
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 使用C实现的一些滤波器函数
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#pragma pack(push)
#pragma pack(1)

/**
 * @brief 卡尔曼滤波器
 *
 */
typedef struct {
    float Q; ///< 过程噪声协方差
    float R; ///< 测量噪声协方差

    float measured_value; ///< 测量值
    float checked_value;  ///< 校验值

    float x_est; ///< 估计值
    float p;     ///< 方差
    float k;     ///< 卡尔曼增益
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *p_kf, float Q, float R);
float KalmanFilter_Refersh(KalmanFilter *p_kf, float measured_value);

/**
 * @brief 均值滤波器
 *
 */
typedef struct {
    int length;  ///< 数据队列长度
    float *fifo; ///< 数据队列
    int fifo_p;  ///< 指针
    float sum;   ///< 队列元素和
    float mean;  ///< 队列元素均值
} MeanFilter;

void MeanFilter_Init(MeanFilter *p_mf, int length, float *p_fifo);
float MeanFilter_Refersh(MeanFilter *p_mf, float measured_value);

/**
 * @brief 联邦卡尔曼滤波器
 *
 */
typedef struct {
    KalmanFilter master;           // 主滤波器
    int local_filter_num;          // 局部滤波器个数
    KalmanFilter *p_local_filters; // 局部滤波器
    float *p_weights;              // 信息分配系数
    float weight_sum;
} FederatedKalmanFilter;

FederatedKalmanFilter_Init(FederatedKalmanFilter *p_fkf, float Q, float R, int num, KalmanFilter *p_lfkf, float *p_weights);
FederatedKalmanFilter_Update(FederatedKalmanFilter *p_fkf, float *input_buf);

#pragma pack(pop)

#endif // !__FILTER_H
