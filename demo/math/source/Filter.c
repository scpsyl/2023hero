#include "Filter.h"

/**
 * @brief ��ʼ����ֵ�˲���
 *
 * @param filter :�˲���
 * @param size :�˲����ߴ�
 */
void Filter_InitAverFilter(AverFilter *filter, uint16_t size)
{
	filter->bufferSize = size;
	for (uint16_t i = 0; i < size; i++)
		filter->buffer[i] = 0;
}
/**
 * @brief ��ʼ����ͨ�˲���
 *
 * @param filter:�˲���
 * @param rate:�˲�ϵ��
 * @return * void
 */
void Filter_InitLowPass(LowPassFilter *filter, float rate)
{
	filter->rate = rate;
	filter->last_data = 0;
}

// ��ֵ�˲�����
float Filter_AverCalc(AverFilter *filter, float newVal)
{
	float sum = 0;
	for (uint16_t i = 0; i < filter->bufferSize - 1; i++) // ��������������
	{
		filter->buffer[i] = filter->buffer[i + 1]; // ��������ǰ��һλ
		sum += filter->buffer[i];				   // ���
	}
	filter->buffer[filter->bufferSize - 1] = newVal; // д��������
	sum += newVal;
	return sum / filter->bufferSize; // �����ֵ
}
// ��ͨ�˲�������
float Filter_LowPass(LowPassFilter *filter, float new_data)
{
	filter->last_data = filter->last_data * (1 - filter->rate) + new_data * filter->rate;
	return filter->last_data;
}

// ��վ�ֵ�˲�������
void Filter_AverClear(AverFilter *filter)
{
	for (uint16_t i = 0; i < filter->bufferSize; i++)
		filter->buffer[i] = 0;
}
