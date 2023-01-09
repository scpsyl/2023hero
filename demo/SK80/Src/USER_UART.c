#include "USER_UART.h"
#include <stdint.h>

/**
 * @file USER_UART.c
 * @brief ͨ�ô�������
 * @retval ����ʹ����Ҫ���к�����չ �ӿ�Ĭ��ȫ������
 */

#define UART1_ENABLE

/**
 * �������FIFO����������
 */

#ifdef UART1_ENABLE

uint8_t UART1_RxBuff[UART1_RX_BUFFSIZE]; // ���ջ�����
uint8_t UART1_TxBuff[UART1_TX_BUFFSIZE]; // ���ͻ�����
UART_Info UART1;

void UART1_InfoInit()
{
	UART1.Rx.length = 0;
	UART1.Rx.RIndex = 0;
	UART1.Rx.WIndex = 0;
	UART1.Tx.length = 0;
	UART1.Tx.RIndex = 0;
	UART1.Tx.WIndex = 0;
	UART1.Tx.Ready = 1;
}

#endif

// Ĭ��ʹ��UART1��ΪSK80ͨ�Ŵ���
bool UART_GetChar(UART_HandleTypeDef *huart, uint8_t *byte)
{
	if (huart == &huart1)
	{
		if (UART1.Rx.length <= 0) // ������û�д�������
			return false;
		else
		{
			// �ض��ж�
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

			*byte = UART1_RxBuff[UART1.Rx.RIndex];
			UART1.Rx.length--;
			UART1.Rx.RIndex++;
			if (UART1.Rx.RIndex >= UART1_RX_BUFFSIZE)
			{
				UART1.Rx.RIndex = 0;
			}
			// �����ж�
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			return true;
		}
	}
	else
		return false;
}
bool UART_GetLastChar(UART_HandleTypeDef *huart, uint8_t *byte)
{
	if (huart == &huart1)
	{
		if (UART1.Rx.length <= 0) // ������û�д�������
			return false;
		else
		{
			// �ض��ж�
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

			// ���UART1.Rx.length-1������
			for (uint16_t index = 0; index < UART1.Rx.length - 1; index++)
			{
				*byte = UART1_RxBuff[UART1.Rx.RIndex];
				UART1.Rx.length--;
				UART1.Rx.RIndex++;
				if (UART1.Rx.RIndex >= UART1_RX_BUFFSIZE)
				{
					UART1.Rx.RIndex = 0;
				}
			}
			*byte = UART1_RxBuff[UART1.Rx.RIndex];
			UART1.Rx.length--;
			UART1.Rx.RIndex++;
			if (UART1.Rx.RIndex >= UART1_RX_BUFFSIZE)
			{
				UART1.Rx.RIndex = 0;
			}
			// �����ж�
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			return true;
		}
	}
	else
		return false;
}
bool UART_GetStr(UART_HandleTypeDef *huart, uint8_t *string)
{
	do
	{
		if (!UART_GetChar(&huart1, string))
			return false;
		string++;
	} while (*(string - 1));
	return true;
}

bool UART_GetDatas(uint8_t *datas, uint16_t length)
{
	if (length > UART1.Rx.length) // ���������ݲ��� �޷���ȡ
		return false;
	while (length--)
	{
		if (!UART_GetChar(&huart1, datas)) // ĳ����ȡʧ��
			return false;
		datas++;
	}
	return true;
}

bool UART_PutChar(UART_HandleTypeDef *huart, uint8_t byte)
{
	if (huart == &huart1)
	{
		if (UART1.Tx.Ready) // ���ڿ���
		{
			HAL_UART_Transmit_IT(&huart1, &byte, sizeof(&byte));
			UART1.Tx.Ready = 0; // busy
			return true;
		}
		else
		{
			if (UART1.Tx.length >= UART1_TX_BUFFSIZE) // ���ͻ���������
				return false;
			else
			{
				// �ض��ж�
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
				UART1_TxBuff[UART1.Tx.WIndex++] = byte;
				if (UART1.Tx.WIndex >= UART1_TX_BUFFSIZE)
					UART1.Rx.WIndex = 0;
				UART1.Tx.length++;
				// �����ж�
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
				return true;
			}
		}
	}
	else
		return false;
}
void UART_ClearRXBuff(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		UART1.Rx.length = 0;
		UART1.Rx.RIndex = 0;
		UART1.Rx.WIndex = 0;
	}
}
