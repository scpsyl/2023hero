/**
 * @file USER_Laser.c
 * @brief SK80��������
 * @retval
 */
#include "USER_Laser.h"
#include "USER_UART.h"
#include "detect.h"
#include "usart.h"

uint16_t sk80_rx_buff[UART1_RX_BUFFSIZE];
uint16_t tof1_tx_buff[UART1_TX_BUFFSIZE];

#define SK80_EN
uint8_t LASER_SetPositionStart(uint8_t start)
{
	if (start == 0 || start == 1)
	{
		UART1_PutChar(0xFA);
		UART1_PutChar(0x04);
		UART1_PutChar(0x08);
		UART1_PutChar(start);
		UART1_PutChar((uint8_t)disNum(0x106 + start));
		HAL_Delay(50);
		return true;
	}
	else
		return false;
}

uint8_t Laser_SetRange(uint8_t range)
{
	switch (range)
	{
	case 5:
	case 10:
	case 30:
	case 50:
	case 80:
	{
		UART1_PutChar(0xFA);
		UART1_PutChar(0x04);
		UART1_PutChar(0x09);
		UART1_PutChar(range);
		UART1_PutChar((uint8_t)disNum(0x107 + range));
		HAL_Delay(50);
		return true;
	}
	default:
		return false;
	}
}

uint8_t Laser_SetFreq(uint8_t freq)
{
	switch (freq)
	{
	case 0:
	case 5:
	case 10:
	case 20:
	{
		UART1_PutChar(0xFA);
		UART1_PutChar(0x04);
		UART1_PutChar(0x0A);
		UART1_PutChar(freq);
		UART1_PutChar((uint8_t)disNum(0x108 + freq));
		HAL_Delay(50);
		return true;
	}
	default:
		return false;
	}
}

uint8_t Laser_SetSolution(uint8_t solu)
{
	switch (solu)
	{
	case 1:
	case 2:
	{
		UART1_PutChar(0xFA);
		UART1_PutChar(0x04);
		UART1_PutChar(0x0C);
		UART1_PutChar(solu);
		UART1_PutChar((uint8_t)disNum(0x109 + solu));
		HAL_Delay(50);
		return true;
	}
	default:
		return false;
	}
}

uint8_t Laser_ChangeDistance(uint8_t signal, uint8_t dist)
{
	switch (signal)
	{
	case 0x2B:
	case 0x2D:
	{
		UART1_PutChar(0xFA);
		UART1_PutChar(0x04);
		UART1_PutChar(0x06);
		UART1_PutChar(signal);
		UART1_PutChar(dist);
		UART1_PutChar((uint8_t)disNum(0x104 + signal + dist));
		HAL_Delay(50);
		return true;
	}
	default:
		return false;
	}
}

uint8_t Laser_MeasuredOnPower(uint8_t start)
{
	switch (start)
	{
	case 0:
	case 1:
	{
		UART1_PutChar(0xFA);
		UART1_PutChar(0x04);
		UART1_PutChar(0x0D);
		UART1_PutChar(start);
		UART1_PutChar((uint8_t)disNum(0x10B + start));
		HAL_Delay(50);
		return true;
	}
	default:
		return false;
	}
}

uint8_t Laser_OpenLight(uint8_t enable)
{
	switch (enable)
	{
	case 0:
	case 1:
	{
		UART1_PutChar(0x80);
		UART1_PutChar(0x06);
		UART1_PutChar(0x05);
		UART1_PutChar(enable);
		UART1_PutChar((uint8_t)disNum(0x8B + enable));
		HAL_Delay(50);
		return true;
	}
	default:
		return false;
	}
}

void Laser_MeasureSignal(void)
{
	UART1_PutChar(0x80);
	UART1_PutChar(0x06);
	UART1_PutChar(0x02);
	UART1_PutChar((uint8_t)disNum(0x88));
	HAL_Delay(50);
}
void Laser_MeasureBroadcast(void)
{
	UART1_PutChar(0xFA);
	UART1_PutChar(0x06);
	UART1_PutChar(0x06);
	UART1_PutChar(0XFA);
	HAL_Delay(50);
}
void Laser_ReadCache()
{
	UART1_PutChar(0x80);
	UART1_PutChar(0x06);
	UART1_PutChar(0x06);
	UART1_PutChar(0XFA);
	HAL_Delay(50);
}

void Laser_MeasureContinous(void)
{
	UART1_PutChar(0x80);
	UART1_PutChar(0x06);
	UART1_PutChar(0x03);
	UART1_PutChar((uint8_t)disNum(0x89));
	HAL_Delay(50);
}
void Laser_SetMeaInte(uint8_t Int)
{
	UART1_PutChar(0xFA);
	UART1_PutChar(0x04);
	UART1_PutChar(0x05);
	UART1_PutChar(Int);
	UART1_PutChar((uint8_t)disNum(0x103 + Int));
	HAL_Delay(50);
}
void Laser_Close(void)
{
	UART1_PutChar(0x80);
	UART1_PutChar(0x04);
	UART1_PutChar(0x02);
	UART1_PutChar((uint8_t)disNum(0x86));
	HAL_Delay(50);
}

void Laser_Init()
{
	MX_USART1_UART_Init();
	LASER_SetPositionStart(1);
	Laser_SetRange(20);
	Laser_SetFreq(20);
	Laser_SetSolution(1);
	Laser_OpenLight(0);
}

uint8_t Laser_GetDistance(uint16_t *distance)
{
	uint8_t flag_start = 0;
	uint8_t temp_data[10];

	// �رռ���
	Laser_OpenLight(1);
	// �������������
	UART_ClearRXBuff(&huart1);
	Laser_MeasureSignal();
	while (1)
	{
		while (!UART_GetChar(&huart1, &flag_start))
			;
		if (flag_start == 0x80)
		{
			// ����
			while (UART1.Rx.length < 10)
				; // ֱ����ȡ������Ϣ����&���ݽ�β
			UART_GetDatas(temp_data, 10);
			if (temp_data[2] == 0x30)
			{
				*distance = 100 * (temp_data[4] - 0x30) + 10 * (temp_data[5] - 0x30) + (temp_data[6] - 0x30);
				HAL_Delay(50);
				// ���´�
				Laser_OpenLight(1);
				return true;
			}
		}
	}
}

void SK80_Init()
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
#ifdef SK80_EN
void USER_USART1_IRQHandler(UART_HandleTypeDef *huart)
{
	static uint8_t rxCnt = 0;
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
	{
		__HAL_UART_CLEAR_NEFLAG(huart);
		sk80_rx_buff[rxCnt++] = huart->Instance->DR;
	}
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		/* clear idle it flag avoid idle interrupt all the time */
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		Laser_GetDistance(sk80_rx_buff);
		rxCnt = 0;

		Detect_Update(DeviceID_TOF);
	}
}

#endif // DEBUG
