/**
 * @file pid_controller.c
 * @author 沈一祺 (qdsyqaaa@gmail.com)
 * @brief 为潜航器提供的PID库
 * @version 0.1
 * @date 2025-04-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "pid_controller.h"

#include <math.h>

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void PID_Init(Pid *pid, float kp, float ki, float kd, float i_limit, float output_limit, float output_rate_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_limit      = fabs(output_limit);
    pid->output_rate_limit = fabs(output_rate_limit);
    pid->i_limit           = fabs(i_limit);

    pid->__prev_error     = 0.0f;
    pid->__prev_output    = 0.0f;
    pid->__prev_integral  = 0.0f;
    pid->__prev_timestamp = pid->get_us(); // 获取微秒时间
}

float Pid_Refresh(Pid *pid, float error)
{
    unsigned long timestamp_now = pid->get_us();
    float ts                    = (timestamp_now - pid->__prev_timestamp) * 1e-6f; // 转换成秒
    if (ts <= 0 || ts > 0.5f)
        ts = 1e-6f;

    // P环处理
    float p_out = pid->kp * error;

    //  散点积分（I环）
    pid->__prev_integral += error * ts;
    pid->__prev_integral = _constrain(pid->__prev_integral, -pid->i_limit, pid->i_limit);
    float i_out        = pid->__prev_integral * pid->ki;

    // D环 散点微分 （微分环）
    float d_out = pid->kd * (error - pid->__prev_error) / ts;

    // 将P，I，D值加起来
    float output = p_out + i_out + d_out;
    output       = _constrain(output, -pid->output_limit, pid->output_limit);

    if (pid->output_rate_limit > 0) {
        // 对PID变化率（加速度）进行限制
        float output_rate = (output - pid->__prev_output) / ts;
        if (output_rate > pid->output_rate_limit)
            output = pid->__prev_output + pid->output_rate_limit * ts;
        else if (output_rate < -pid->output_rate_limit)
            output = pid->__prev_output - pid->output_rate_limit * ts;
    }

    pid->__prev_output    = output;
    pid->__prev_error     = error;
    pid->__prev_timestamp = timestamp_now;
    return output;
}

void PID_Set(Pid *pid, float kp, float ki, float kd, float i_limit, float output_limit, float output_rate_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_limit      = fabs(output_limit);
    pid->output_rate_limit = fabs(output_rate_limit);
    pid->i_limit           = fabs(i_limit);

    pid->__prev_error     = 0.0f;
    pid->__prev_output    = 0.0f;
    pid->__prev_integral  = 0.0f;
    pid->__prev_timestamp = pid->get_us(); // 获取毫秒时间
}
