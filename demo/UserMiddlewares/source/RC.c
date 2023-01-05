/**
 * @file RC.c
 * @author Screeps
 * @brief ң����
 * @version 0.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "RC.h"
#include "USER_CAN.h"
#include "Detect.h"
#include "UserFreertos.h"
#define EN_RC_TASK		 // ʹ������
uint8_t usart3RxBuf[30]; // ����3������
RC_TypeDef rcInfo = {0}; // ң������Ϣ
Key keyList[KEY_NUM];	 // �����б�(�������п��ü��̰�����������Ҽ�)

// �ڲ�����

void RC_InitKeyJudgeTime(uint32_t key, uint16_t clickDelay, uint16_t longPressDelay);
void RC_InitKeys(void);
void RC_UpdateKeys(void);
void Task_RC_Callback(void);

// ң������ʼ��
void RC_Init()
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	RC_InitKeys(); // ��ʼ������
}

// ������������
void RC_ParseUsart(uint8_t *buff)
{

	rcInfo.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	rcInfo.ch1 -= 1024;
	rcInfo.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	rcInfo.ch2 -= 1024;
	rcInfo.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	rcInfo.ch3 -= 1024;
	rcInfo.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	rcInfo.ch4 -= 1024;

	/* prevent remote control zero deviation */
	if (rcInfo.ch1 <= 5 && rcInfo.ch1 >= -5)
		rcInfo.ch1 = 0;
	if (rcInfo.ch2 <= 5 && rcInfo.ch2 >= -5)
		rcInfo.ch2 = 0;
	if (rcInfo.ch3 <= 5 && rcInfo.ch3 >= -5)
		rcInfo.ch3 = 0;
	if (rcInfo.ch4 <= 5 && rcInfo.ch4 >= -5)
		rcInfo.ch4 = 0;

	rcInfo.left = ((buff[5] >> 4) & 0x000C) >> 2; // sw1   �м���3���ϱ���1���±���2
	rcInfo.right = (buff[5] >> 4) & 0x0003;		  // sw2
	/*
	 if ((abs(rc->ch1) > 660) || \
		 (abs(rc->ch2) > 660) || \
		 (abs(rc->ch3) > 660) || \
		 (abs(rc->ch4) > 660))
	 {
	   memset(rc, 0, sizeof(struct rc));
	   return ;
	 }
   */
	rcInfo.mouse.x = buff[6] | (buff[7] << 8); // x axis
	rcInfo.mouse.y = buff[8] | (buff[9] << 8);
	rcInfo.mouse.z = buff[10] | (buff[11] << 8);

	rcInfo.mouse.l = buff[12];
	rcInfo.mouse.r = buff[13];

	rcInfo.kb.key_code = buff[14] | buff[15] << 8; // key borad code
	rcInfo.wheel = (buff[16] | buff[17] << 8) - 1024;
}

// ����3�жϻص�
void USER_USART3_IRQHandler(UART_HandleTypeDef *huart)
{
	static uint16_t rxCnt = 0;

	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
	{
		usart3RxBuf[rxCnt++] = huart3.Instance->DR;
	}

	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
	{
		/* clear idle it flag avoid idle interrupt all the time */
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);

		if (rxCnt == 18u)
		{
			RC_ParseUsart(usart3RxBuf); // ������յ�������
		}
		Detect_Update(DeviceID_RC);
		rxCnt = 0;
	}
}

// ע��һ�������ص�
void RC_Register(uint32_t key, KeyCombineType combine, KeyEventType event, KeyCallbackFunc func)
{
	// Ѱ��Ҫ�����İ���
	for (uint8_t index = 0; index < KEY_NUM; index++)
	{
		if (key & (0x01 << index))
		{
			// ���ݰ����¼�ע��ص�
			switch (event)
			{
			case KeyEvent_OnClick:
				keyList[index].onClickCb.combineKey[keyList[index].onClickCb.number] = combine;
				keyList[index].onClickCb.func[keyList[index].onClickCb.number] = func;
				keyList[index].onClickCb.number++;
				break;
			case KeyEvent_OnLongPress:
				keyList[index].onLongCb.combineKey[keyList[index].onLongCb.number] = combine;
				keyList[index].onLongCb.func[keyList[index].onLongCb.number] = func;
				keyList[index].onLongCb.number++;
				break;
			case KeyEvent_OnDown:
				keyList[index].onDownCb.combineKey[keyList[index].onDownCb.number] = combine;
				keyList[index].onDownCb.func[keyList[index].onDownCb.number] = func;
				keyList[index].onDownCb.number++;
				break;
			case KeyEvent_OnUp:
				keyList[index].onUpCb.combineKey[keyList[index].onUpCb.number] = combine;
				keyList[index].onUpCb.func[keyList[index].onUpCb.number] = func;
				keyList[index].onUpCb.number++;
				break;
			case KeyEvent_OnPressing:
				keyList[index].onPressCb.combineKey[keyList[index].onPressCb.number] = combine;
				keyList[index].onPressCb.func[keyList[index].onPressCb.number] = func;
				keyList[index].onPressCb.number++;
				break;
			}
		}
	}
}

// ��ʼ��һ���������ж�ʱ��(��λID�������ж�ʱ�䣬�����ж�ʱ��)
void RC_InitKeyJudgeTime(uint32_t key, uint16_t clickDelay, uint16_t longPressDelay)
{
	for (uint8_t i = 0; i < 18; i++)
	{
		if (key & (0x01 << i))
		{
			keyList[i].clickDelayTime = clickDelay;
			keyList[i].longPressTime = longPressDelay;
		}
	}
}

/**
 * @brief ��ʼ�����а���
 *
 */
void RC_InitKeys()
{
	RC_InitKeyJudgeTime(Key_All, 50, 500);
}

/**
 * @brief ���°���״̬
 *
 */
void RC_UpdateKeys(void)
{
	static uint32_t lastUpdateTime;
	uint32_t presentTime = HAL_GetTick();

	// �����ϼ�
	KeyCombineType combineKey = CombineKey_None;
	if (rcInfo.kb.bit.CTRL)
		combineKey = CombineKey_Ctrl;
	else if (rcInfo.kb.bit.SHIFT)
		combineKey = CombineKey_Shift;

	for (uint8_t key = 0; key < 18; key++)
	{
		// ��ȡ����״̬
		uint8_t thisState = 0;
		if (key < 16)
			thisState = (rcInfo.kb.key_code & (0x01 << key)) ? 1 : 0; // ȡ�����̶�Ӧλ
		else if (key == 16)
			thisState = rcInfo.mouse.l;
		else if (key == 17)
			thisState = rcInfo.mouse.r;

		uint16_t lastPressTime = lastUpdateTime - keyList[key].startPressTime; // �ϴθ���ʱ���µ�ʱ��
		uint16_t pressTime = presentTime - keyList[key].startPressTime;		   // ��ǰ���µ�ʱ��

		// ����״̬�ж�
		/*******���µ�һ˲��********/
		if (!keyList[key].lastState && thisState)
		{
			keyList[key].startPressTime = presentTime; // ��¼����ʱ��
			keyList[key].isPressing = 1;

			// ����ִ�лص�
			for (uint8_t i = 0; i < keyList[key].onDownCb.number; i++)
				if (keyList[key].onDownCb.combineKey[i] == combineKey) // ������ϼ�����
					keyList[key].onDownCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnDown);
		}
		/*******�ɿ���һ˲��********/
		else if (keyList[key].lastState && !thisState)
		{
			keyList[key].isLongPressed = 0;
			keyList[key].isPressing = 0;

			// ����̧��
			keyList[key].isUp = 1;
			// ����ִ�лص�
			for (uint8_t i = 0; i < keyList[key].onUpCb.number; i++)
				if (keyList[key].onUpCb.combineKey[i] == combineKey) // ������ϼ�����
					keyList[key].onUpCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnUp);

			// �����ж�
			if (pressTime > keyList[key].clickDelayTime && pressTime < keyList[key].longPressTime)
			{
				keyList[key].isClicked = 1;
				// ����ִ�лص�
				for (uint8_t i = 0; i < keyList[key].onClickCb.number; i++)
					if (keyList[key].onClickCb.combineKey[i] == combineKey) // ������ϼ�����
						keyList[key].onClickCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnClick);
			}
		}
		/*******������������********/
		else if (keyList[key].lastState && thisState)
		{
			// ִ��һֱ���µ��¼��ص�
			for (uint8_t i = 0; i < keyList[key].onPressCb.number; i++)
				if (keyList[key].onPressCb.combineKey[i] == combineKey) // ������ϼ�����
					keyList[key].onPressCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnPressing);

			// �����ж�
			if (lastPressTime <= keyList[key].longPressTime && pressTime > keyList[key].longPressTime)
			{
				keyList[key].isLongPressed = 1;
				// ����ִ�лص�
				for (uint8_t i = 0; i < keyList[key].onLongCb.number; i++)
					if (keyList[key].onLongCb.combineKey[i] == combineKey) // ������ϼ�����
						keyList[key].onLongCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnLongPress);
			}
			else
				keyList[key].isLongPressed = 0;
		}
		/*******���������ɿ�********/
		else
		{
			keyList[key].isClicked = 0;
			keyList[key].isLongPressed = 0;
			keyList[key].isUp = 0;
		}

		keyList[key].lastState = thisState; // ��¼����״̬
	}

	lastUpdateTime = presentTime; // ��¼�����¼�
}

// ң������ص�
void Task_RC_Callback()
{
	// ���°���״̬
	RC_UpdateKeys();
	/**********�����������*********************/
	if (rcInfo.right == 2) // ң�����Ҳ��뿪�����£���ͣ
	{
		osThreadResume(ErrorTaskHandle);					 // �ָ��������� ������������
		HAL_Delay(1);										 // ��ֹ����ո�����
		USER_CAN_SetMotorCurrent(&hcan1, 0x200, 0, 0, 0, 0); // �ضϵ��
		USER_CAN_SetMotorCurrent(&hcan2, 0x200, 0, 0, 0, 0);
		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hcan1, 0x1ff, 0, 0, 0, 0);
		USER_CAN_SetMotorCurrent(&hcan2, 0x1ff, 0, 0, 0, 0);
	}
}

/************************freertos����****************************/
#ifdef EN_RC_TASK // ʹ������
void os_RCCallback(void const *argument)
{

	for (;;)
	{
		Task_RC_Callback();
		osDelay(10);
	}
}
#endif
