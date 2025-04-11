/**
 * @file pid_controller.h
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 为潜航器提供的PID库
 * @version 0.1
 * @date 2025-04-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __PID_H
#define __PID_H


typedef float (*FUNC_GET_US)();

typedef struct {
    float kp; // 比例增益
    float ki; // 积分增益
    float kd; // 微分增益
    float output_limit;
    float output_rate_limit;
    float i_limit;

    float __prev_error;             // 最后跟踪误差
    float __prev_output;            // 最后一个pid输出值
    float __prev_integral;          // 最后一个积分分量
    unsigned long __prev_timestamp; // 上次执行的时间戳

    FUNC_GET_US get_us;
} Pid;

void PID_Init(Pid *pid, float kp, float ki, float kd, float i_limit, float output_limit, float output_rate_limit);
void PID_Set(Pid *pid, float kp, float ki, float kd, float i_limit, float output_limit, float output_rate_limit);
float Pid_Refresh(Pid *pid, float error);

#endif
