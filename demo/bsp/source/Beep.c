#include "Beep.h"
#include "myQueue.h"
#include "userFreertos.h"
#include "tim.h"

#define SOFTWARE_DELAY(t)       \
	do                          \
	{                           \
		uint32_t i = (t)*12000; \
		while (i--)             \
			;                   \
	} while (0) // �����ʱ

#define MAX_NOTE_QUEUE_SIZE 50

Queue noteQueue;						// �������У���Ž�Ҫ���ŵ�����
Note noteQueueBuf[MAX_NOTE_QUEUE_SIZE]; // ��������

// ��������ʼ��
void Beep_Init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// ���г�ʼ��
	Queue_Init(&noteQueue, MAX_NOTE_QUEUE_SIZE);
	Queue_AttachBuffer(&noteQueue, noteQueueBuf, sizeof(Note));
}

// ����������ָ������(us)ռ�ձ�50%�ķ���
void Beep_Play(uint16_t period)
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	if (period != 0)
	{
		__HAL_TIM_SetAutoreload(&htim4, period);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, period / 2);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	}
}

// ����һ������
void Beep_PlayOneNote(Note *note)
{
	// ���
	Queue_Enqueue(&noteQueue, note);
	xTaskNotifyGive(BeepTaskHandle); // ֪ͨbeep����ʼ
}

// ����һ������
void Beep_PlayNotes(Note notes[], uint8_t noteNum)
{
	// ���
	for (int i = 0; i < noteNum; i++)
		Queue_Enqueue(&noteQueue, &notes[i]);
	xTaskNotifyGive(BeepTaskHandle);
}

// ����ʽ������������(��Ӧ�����������������ʱ����)
void Beep_Block_PlayAll()
{
	while (!Queue_IsEmpty(&noteQueue)) // ���Ŷ�������������
	{
		Note *note = Queue_Dequeue(&noteQueue);
		Beep_Play(note->period);
		SOFTWARE_DELAY(note->duration); // ����ʽ��ʱ��ʹ�������ʱ��ֹ���ж��е���
	}
	Beep_Play(T_None);
}

// ����������ص�
void Task_Beep_Callback()
{
	if (Queue_IsEmpty(&noteQueue)) // û������Ҫ������ֹͣ����
	{
		Beep_Play(T_None);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // �ȴ�����֪ͨ
	}
	else
	{
		// ���Ŷ�ͷ�������ȴ�ָ��ʱֵ
		Note *note = Queue_Dequeue(&noteQueue);
		Beep_Play(note->period);
		osDelay(note->duration);
	}
}

/************************freertos����***************************/
void os_BeepCallback(void const *argument)
{
	// ���������������� [do mi so do(��)]
	Beep_PlayNotes((Note[]){{T_M2, D_Sixteenth},
							{T_M4, D_Sixteenth},
							{T_M6, D_Sixteenth},
							{T_H2, D_Sixteenth},
							{T_None, D_Quarter}},
				   5);
	for (;;)
	{
		Task_Beep_Callback();
		osDelay(20);
	}
}
