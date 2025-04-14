# STM32_UUV_UTILS
useful c codes for underwater robot control based on stm32 hal



## 坐标系变换 Coordinate Conversion

通过利用机器人齐次矩阵理论实现的机器人坐标系操作，以下是使用指导。

创建`BaseSys`结构体，并用`BaseSys_Init`对其初始化。

```c
BaseSys bs;
BaseSys_Init(&bs);
```

可通过以下函数对坐标描述系统赋值。

```C
void BaseSys_SetbyParams(BaseSys *p_bs, float x, float y, float z, float rx, float ry, float rz);
void BaseSys_SetbyArray(BaseSys *p_bs, float *arr);
void BaseSys_SetbyVector6d(BaseSys *p_bs, Vector6d *p_vector);
void BaseSys_SetbyBaseSys(BaseSys *p_bs, BaseSys *p_bs_src);
```

提供了对坐标描述系统进行加减操作的函数。

```C
// 相加操作，将bs1、bs2相加的结果赋予bs_add_ret
BaseSys bs_add_ret;
BaseSysAdd(&bs1,&bs2,&bs_add_ret);

// 相减操作，将bs1减bs2的结果赋予bs_minus_ret
BaseSys bs_minus_ret;
BaseSysMinus(&bs1,&bs2,&bs_minus_ret);
```

也提供了坐标系变换操作支持。

```C
// 世界坐标系变换至机器人坐标系,将世界坐标系中的坐标(bs_in_world)转换为机器人坐标系中坐标(bs_out)
BaseSys bs_out;
World2Base(&bs_robot,&bs_in_world,&bs_out);

// 机器人坐标系至世界坐标系变换,将机器人坐标系中坐标(bs_in_robot)转换为世界坐标系中的坐标(bs_out)
BaseSys bs_out;
Base2World(&bs_robot,&bs_in_robot,&bs_out);

// 机器人坐标系之间变换,将机器人坐标系1中坐标(bs_in_robot1)转换为机器人坐标系2中的坐标(bs_out)
BaseSys bs_out;
Base2Base(&bs_robot1,&bs_robot2,&bs_in_robot1,&bs_out);
```



## 先入先出队列 Fifo

使用C实现的一个先入先出队列，以便于处理不定长串口数据，以下是使用指导。

创建`Fifo`结构体，并使用`Fifo_Init`对结构体初始化。需要创建两个数组，一个用于包含帧头信息，另一个用于包含帧尾信息。

```C
Fifo rec_buf;
uint8_t head[HEAD_LENGTH] = {0xfa,0xaf};
uint8_t end[END_LENGTH] = {0xfb,0xbf};
Fifo_Init(&rec_buf,MAX_LENGTH,HEAD_LENGTH,END_LENGTH,head,end);
```

预设了两种触发队列更新的方式。一种为外部更新，一种为直接更新。

当使用外部的函数对队列内容进行更新时(如使用hal的中断接收函数`HAL_UART_Receive_IT`时)，可使用`Fifo_RefreshExternal`函数对队列状态进行更新。

```C
FIFO_RESAULT ret = Fifo_RefreshExternal(&rec_buf);
```

也可以使用`Fifo_RefreshInternal`函数对单个字节的输入进行处理：

```C
uint8_t input;
FIFO_RESAULT ret = Fifo_RefreshInternal(&rec_buf,input);
```

两个函数都会返回对数据队列的`FIFO_RESAULT`类型接收结果。

```C
typedef enum {
    FIFO_RECEIVED        = 0, ///< 完成接收
    FIFO_FINISH_ONE      = 1, ///< 完成一个字节的接收
    FIFO_HEAD_CHECKED    = 2, ///< 头校验完成
    FIFO_END_CHECKED     = 3, ///< 尾校验完成
    FIFO_OVERFLOW        = 4, ///< 接收溢出
    FIFO_REFERSH_TIMEOUT = 5  ///< 更新程序超时退出
} FIFO_RESAULT;
```



## 滤波器 Filter

使用C编写的一些简单的滤波器，包含卡尔曼滤波器、均值滤波器、联邦卡尔曼滤波器。

### 卡尔曼滤波器 Kalman Filter

创建`KalmanFilter`结构体，并使用`KalmanFilter_Init`函数初始化结构体并输入参数。

```C
KalmanFilter kf;
KalmanFilter_Init(&kf,Q,R); // Q 过程噪声协方差,R 测量噪声协方差
```

使用`KalmanFilter_Refersh`函数对结构体进行更新。

```C
float ret = KalmanFilter_Refersh(&kf,measured_value);// measured_value 为测量值
```



### 均值滤波器 Mean Filter

值得注意的是均值滤波器同时也是个低通滤波器，有时候可以配合DMA操作实现高性能的低通滤波器。其截止频率为
$$
f_c = \frac{0.443*{f}_{sample}}{N}
$$
均值滤波器的使用方法如下。

创建`MeanFilter`结构体，并使用`MeanFilter_Init`函数初始化结构体。需要在创建一个`float`类型数组以充当其数据缓冲。

```C
MeanFilter mf;
float mf_buf[LENGTH];
MeanFilter_Init(&mf,LENGTH,mf_buf);
```

使用`MeanFilter_Refersh`函数对输入数据进行处理

```C
float measured_value;
float ret = MeanFilter_Refersh(&mf,measured_value);
```



### 联邦卡尔曼滤波器 Federated Kalman Filter

创建`FederatedKalmanFilter`结构体，并使用`FederatedKalmanFilter_Init`函数初始化结构体。需要额外创建并初始化`KalmanFilter`类型的数组，以充当其局部滤波器组。并需要一个`float`类型数组以描述各局部滤波器权重。

```C
FederatedKalmanFilter fkf;
KalmanFilter local_kf[LOCCAL_FILTER_NUM];
KalmanFilter_Init(&local_kf[0],Q0,R0);
.
.
.
KalmanFilter_Init(&local_kf[LOCCAL_FILTER_NUM - 1],Qn,Rn);
float weights[LOCCAL_FILTER_NUM];
FederatedKalmanFilter_Init(&fkf,Q,R,LOCCAL_FILTER_NUM,local_kf,weights);
```

更新时使用`FederatedKalmanFilter_Update`函数，需使用一个`float`类型数组作为对应各个局部滤波器的输入。

```C
float measured_values[LOCCAL_FILTER_NUM];
float ret = FederatedKalmanFilter_Update(&fkf,measured_values);
```



## PID控制器 Pid Controller

若需要其中微分项生效，需要注意奈奎斯特采样定律，保证PID控制器的采样频率小于传感器采样频率的一半。以下为使用教程。

创建`Pid`类型结构体，并使用`PID_Init`函数初始化。需要绑定微秒计时函数。

```C
Pid pid;
PID_Init(&pid,kp, ki, kd, i_limit, output_limit, output_rate_limit);
pid.get_us = get_us_counter;

uint32_t get_us_counter(void)
{
    ...
    return us_counter;
}
```

使用`Pid_Refresh`函数更新PID控制器，并获取输出值。

```C
float error
float output = Pid_Refresh(&pid,error);
```



## 动力分配 Power Distribution

包含两个部分，一部分利用数学模型将所需推进器推力转换为所需转速信号；一部分将潜器所需推力与推力矩依据推力矩阵分配给各推进器。

### 推力曲线 Thrust Cruve

采用以下的推力-转速模型,，其中 $t_2$ 用于描述速度导致的推力衰减。
$$
F_{thrust} = t_1 \omega \left | \omega \right |  - t_2 v \left | \omega \right |
$$
创建`ThrustCruve`结构体，并使用`ThrustCruve_Init`初始化。

```C
ThrustCruve tc;
float t_1;
float t_2;
float t_max; // 推力上限
float t_min; // 反向推力上限,取负值
float s_middle; // 信号中值
ThrustCruve_Init(&tc,t1,t2,t_max,t_min,s_middle);
```

使用`ThrustCruve_GetSignal`函数，输入所需推力与推进器处水流流速以获取对应转速信号值。

```C
float signal = ThrustCruve_GetSignal(&tc,thrust,speed);
```

同时提供了辅助程序以计算推进器处水流流速。

创建`BaseSys`结构体以描述推进器朝向，并使用`MotorAtt_Init`初始化该结构体。

```C
BaseSys motor_att;
MotorAtt_Init(&motor_att,rx,ry,rz);
```

使用`float`类型数组分别描述潜器速度与水流速度，使用`MotorAtt_GetSpd`函数获取推进器处水流流速。

```C
float base_spd_xyz[3];
float fluid_spd_xyz[3];
float spd = MotorAtt_GetSpd(&motor_att,base_spd_xyz,fluid_spd_xyz);
```



### 推力分配 Thrust Allocate

依据推力矩阵将潜器所需推力与推力矩分配给各推进器。

创建`ThrustAllocate`结构体，并使用相应函数初始化。

```C
ThrustAllocate ta;

// 直接输入参数初始化
float x_cf, y_cf, z_cf, rx_cf, ry_cf, rz_cf;
ThrustAllocate_InitfromParams(&ta,x_cf, y_cf, z_cf, rx_cf, ry_cf, rz_cf);

// 使用数组初始化
float params[6];
ThrustAllocate_InitfromArray(&ta,params);
```

输入所需推力与推力矩，获取所需推进器的推力。

```C
// 依次输入所需的推力与推力矩
float x_thrust, y_thrust, z_thrust, x_torque, y_torque, z_torque;
float thrust = ThrustAllocate_AllocatefromParams(&ta,x_thrust, y_thrust, z_thrust, x_torque, y_torque, z_torque);

// 使用 Vector6d 结构体传参
Vector6d t;
float thrust = ThrustAllocate_AllocatefromVector6d(&ta,&t);
```
