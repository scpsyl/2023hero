/**
 * @file RC.c
 * @author Screeps
 * @brief 遥控器
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
#define EN_RC_TASK		 // 使能任务
uint8_t usart3RxBuf[30]; // 串口3缓冲区
RC_TypeDef rcInfo = {0}; // 遥控器信息
Key keyList[KEY_NUM];	 // 按键列表(包含所有可用键盘按键和鼠标左右键)

// 内部函数

void RC_InitKeyJudgeTime(uint32_t key, uint16_t clickDelay, uint16_t longPressDelay);
void RC_InitKeys(void);
void RC_UpdateKeys(void);
void Task_RC_Callback(void);

// 遥控器初始化
void RC_Init()
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	RC_InitKeys(); // 初始化按键
}

// 解析串口数据
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

	rcInfo.left = ((buff[5] >> 4) & 0x000C) >> 2; // sw1   中间是3，上边是1，下边是2
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

// 串口3中断回调
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
			RC_ParseUsart(usart3RxBuf); // 解码接收到的数据
		}
		Detect_Update(DeviceID_RC);
		rxCnt = 0;
	}
}

// 注册一个按键回调
void RC_Register(uint32_t key, KeyCombineType combine, KeyEventType event, KeyCallbackFunc func)
{
	// 寻找要操作的按键
	for (uint8_t index = 0; index < KEY_NUM; index++)
	{
		if (key & (0x01 << index))
		{
			// 根据按键事件注册回调
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

// 初始化一个按键的判定时间(键位ID，单击判定时间，长按判定时间)
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
 * @brief 初始化所有按键
 *
 */
void RC_InitKeys()
{
	RC_InitKeyJudgeTime(Key_All, 50, 500);
}

/**
 * @brief 更新按键状态
 *
 */
void RC_UpdateKeys(void)
{
	static uint32_t lastUpdateTime;
	uint32_t presentTime = HAL_GetTick();

	// 检查组合键
	KeyCombineType combineKey = CombineKey_None;
	if (rcInfo.kb.bit.CTRL)
		combineKey = CombineKey_Ctrl;
	else if (rcInfo.kb.bit.SHIFT)
		combineKey = CombineKey_Shift;

	for (uint8_t key = 0; key < 18; key++)
	{
		// 读取按键状态
		uint8_t thisState = 0;
		if (key < 16)
			thisState = (rcInfo.kb.key_code & (0x01 << key)) ? 1 : 0; // 取出键盘对应位
		else if (key == 16)
			thisState = rcInfo.mouse.l;
		else if (key == 17)
			thisState = rcInfo.mouse.r;

		uint16_t lastPressTime = lastUpdateTime - keyList[key].startPressTime; // 上次更新时按下的时间
		uint16_t pressTime = presentTime - keyList[key].startPressTime;		   // 当前按下的时间

		// 按键状态判定
		/*******按下的一瞬间********/
		if (!keyList[key].lastState && thisState)
		{
			keyList[key].startPressTime = presentTime; // 记录按下时间
			keyList[key].isPressing = 1;

			// 依次执行回调
			for (uint8_t i = 0; i < keyList[key].onDownCb.number; i++)
				if (keyList[key].onDownCb.combineKey[i] == combineKey) // 符合组合键条件
					keyList[key].onDownCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnDown);
		}
		/*******松开的一瞬间********/
		else if (keyList[key].lastState && !thisState)
		{
			keyList[key].isLongPressed = 0;
			keyList[key].isPressing = 0;

			// 按键抬起
			keyList[key].isUp = 1;
			// 依次执行回调
			for (uint8_t i = 0; i < keyList[key].onUpCb.number; i++)
				if (keyList[key].onUpCb.combineKey[i] == combineKey) // 符合组合键条件
					keyList[key].onUpCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnUp);

			// 单击判定
			if (pressTime > keyList[key].clickDelayTime && pressTime < keyList[key].longPressTime)
			{
				keyList[key].isClicked = 1;
				// 依次执行回调
				for (uint8_t i = 0; i < keyList[key].onClickCb.number; i++)
					if (keyList[key].onClickCb.combineKey[i] == combineKey) // 符合组合键条件
						keyList[key].onClickCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnClick);
			}
		}
		/*******按键持续按下********/
		else if (keyList[key].lastState && thisState)
		{
			// 执行一直按下的事件回调
			for (uint8_t i = 0; i < keyList[key].onPressCb.number; i++)
				if (keyList[key].onPressCb.combineKey[i] == combineKey) // 符合组合键条件
					keyList[key].onPressCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnPressing);

			// 长按判定
			if (lastPressTime <= keyList[key].longPressTime && pressTime > keyList[key].longPressTime)
			{
				keyList[key].isLongPressed = 1;
				// 依次执行回调
				for (uint8_t i = 0; i < keyList[key].onLongCb.number; i++)
					if (keyList[key].onLongCb.combineKey[i] == combineKey) // 符合组合键条件
						keyList[key].onLongCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnLongPress);
			}
			else
				keyList[key].isLongPressed = 0;
		}
		/*******按键持续松开********/
		else
		{
			keyList[key].isClicked = 0;
			keyList[key].isLongPressed = 0;
			keyList[key].isUp = 0;
		}

		keyList[key].lastState = thisState; // 记录按键状态
	}

	lastUpdateTime = presentTime; // 记录更新事件
}

// 遥控任务回调
void Task_RC_Callback()
{
	// 更新按键状态
	RC_UpdateKeys();
	/**********特殊情况处理*********************/
	if (rcInfo.right == 2) // 遥控器右拨码开关向下，急停
	{
		osThreadResume(ErrorTaskHandle);					 // 恢复错误任务 饿死其他任务
		HAL_Delay(1);										 // 防止邮箱刚刚塞满
		USER_CAN_SetMotorCurrent(&hcan1, 0x200, 0, 0, 0, 0); // 关断电机
		USER_CAN_SetMotorCurrent(&hcan2, 0x200, 0, 0, 0, 0);
		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hcan1, 0x1ff, 0, 0, 0, 0);
		USER_CAN_SetMotorCurrent(&hcan2, 0x1ff, 0, 0, 0, 0);
	}
}

/************************freertos任务****************************/
#ifdef EN_RC_TASK // 使能任务
void os_RCCallback(void const *argument)
{

	for (;;)
	{
		Task_RC_Callback();
		osDelay(10);
	}
}
#endif
