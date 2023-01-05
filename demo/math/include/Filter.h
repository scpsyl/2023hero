#ifndef _FILTER_H_
#define _FILTER_H_

#include "stdint.h"

// 均值滤波器最大允许的缓冲区长度
#define AVER_FILTER_MAX_SIZE 100

// 低通滤波结构体
typedef struct
{
	float rate;
	float last_data;
} LowPassFilter;

// 一阶卡尔曼滤波结构体
typedef struct
{
	float X_last; // 上一时刻的最优结果  X(k|k-1)
	float X_pre;  // 当前时刻的预测结果  X(k|k-1)
	float X_now;  // 当前时刻的最优结果  X(k|k)
	float P_pre;  // 当前时刻预测结果的协方差  P(k|k-1)
	float P_now;  // 当前时刻最优结果的协方差  P(k|k)
	float P_last; // 上一时刻最优结果的协方差  P(k-1|k-1)
	float kg;	  // kalman增益
	float Q;
	float R;
} KalmanFilter;

// 均值滤波器结构体
typedef struct
{
	float buffer[AVER_FILTER_MAX_SIZE];
	uint16_t bufferSize;
} AverFilter;
void Filter_InitLowPass(LowPassFilter *filter, float rate);
void Filter_InitAverFilter(AverFilter *filter, uint16_t size);
float Filter_AverCalc(AverFilter *filter, float newVal);
float Filter_LowPass(LowPassFilter *filter, float new_data);
void Filter_AverClear(AverFilter *filter);

#endif
