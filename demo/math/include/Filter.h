#ifndef _FILTER_H_
#define _FILTER_H_

#include "stdint.h"

// ��ֵ�˲����������Ļ���������
#define AVER_FILTER_MAX_SIZE 100

// ��ͨ�˲��ṹ��
typedef struct
{
	float rate;
	float last_data;
} LowPassFilter;

// һ�׿������˲��ṹ��
typedef struct
{
	float X_last; // ��һʱ�̵����Ž��  X(k|k-1)
	float X_pre;  // ��ǰʱ�̵�Ԥ����  X(k|k-1)
	float X_now;  // ��ǰʱ�̵����Ž��  X(k|k)
	float P_pre;  // ��ǰʱ��Ԥ������Э����  P(k|k-1)
	float P_now;  // ��ǰʱ�����Ž����Э����  P(k|k)
	float P_last; // ��һʱ�����Ž����Э����  P(k-1|k-1)
	float kg;	  // kalman����
	float Q;
	float R;
} KalmanFilter;

// ��ֵ�˲����ṹ��
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
