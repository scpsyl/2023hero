#include "Filter.h"

/**
 * @brief 初始化均值滤波器
 *
 * @param filter :滤波器
 * @param size :滤波器尺寸
 */
void Filter_InitAverFilter(AverFilter *filter, uint16_t size)
{
	filter->bufferSize = size;
	for (uint16_t i = 0; i < size; i++)
		filter->buffer[i] = 0;
}
/**
 * @brief 初始化低通滤波器
 *
 * @param filter:滤波器
 * @param rate:滤波系数
 * @return * void
 */
void Filter_InitLowPass(LowPassFilter *filter, float rate)
{
	filter->rate = rate;
	filter->last_data = 0;
}

// 均值滤波计算
float Filter_AverCalc(AverFilter *filter, float newVal)
{
	float sum = 0;
	for (uint16_t i = 0; i < filter->bufferSize - 1; i++) // 遍历整个缓冲区
	{
		filter->buffer[i] = filter->buffer[i + 1]; // 所有数据前移一位
		sum += filter->buffer[i];				   // 求和
	}
	filter->buffer[filter->bufferSize - 1] = newVal; // 写入新数据
	sum += newVal;
	return sum / filter->bufferSize; // 计算均值
}
// 低通滤波器计算
float Filter_LowPass(LowPassFilter *filter, float new_data)
{
	filter->last_data = filter->last_data * (1 - filter->rate) + new_data * filter->rate;
	return filter->last_data;
}

// 清空均值滤波缓冲区
void Filter_AverClear(AverFilter *filter)
{
	for (uint16_t i = 0; i < filter->bufferSize; i++)
		filter->buffer[i] = 0;
}
