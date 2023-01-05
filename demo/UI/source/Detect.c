#include "Detect.h"
#include "USER_CAN.h"
// #include "Beep.h"
#include "Vision.h"
#include "Gimbal.h"
#include "judge.h"

/****�ڲ���������****/
// Ĭ�ϵ��ߴ�����
void Detect_DefaultLostHandler(uint8_t deviceID);
// Ĭ�����ߴ�����
void Detect_DefaultRecoverHandler(uint8_t deviceID);
// ��ʼ�������豸
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void));

// �豸������Ϣ�б�
DetectDevice detectList[DETECT_DEVICE_NUM];

// ����ָ��id�豸��״̬
void Detect_Update(uint8_t deviceID)
{
	detectList[deviceID].lastRecieveTime = HAL_GetTick();
}

// Ĭ�ϵ��ߴ���������lostFunc==NULL�����
void Detect_DefaultLostHandler(uint8_t deviceID)
{
	/*//�����������ȳ��죬�ٶ���device��Ӧ��������(��λ��)
	Beep_PlayNotes((Note[2]){{T_H5,500},{T_None,500}},2);
	deviceID+=1;
	while(deviceID)
	{
		if(deviceID&0x01) Beep_PlayNotes((Note[2]){{T_M5,200},{T_None,100}},2);//������Ӧ1
		else Beep_PlayNotes((Note[2]){{T_M1,200},{T_None,100}},2);//������Ӧ0
		deviceID>>=1;
	}
	Beep_PlayOneNote(&(Note){T_None,1000});*/
}

// Ĭ�ϵ��ߴ���������recoverFunc==NULL�����
void Detect_DefaultRecoverHandler(uint8_t deviceID)
{
}

// ��ʼ��һ���豸�ĵ��߼����Ϣ(�豸id�����ݽ�������������ߴ���ص�����)
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void))
{
	detectList[deviceID].maxInterval = maxInterval;
	detectList[deviceID].isLost = 0;
	detectList[deviceID].lostFunc = lostFunc;
	detectList[deviceID].recoverFunc = recoverFunc;
}

// ��ʼ�������豸�ĵ��߼����Ϣ
void Detect_InitAll()
{
	Detect_InitDevice(DeviceID_ChassisMotor1, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor2, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor3, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor4, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_YawMotor, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_PitchMotor, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_FricMotor1, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_FricMotor2, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_TrigMotor, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_RC, 100, NULL, NULL);

	Detect_InitDevice(DeviceID_Judge, 500, Judge_UartLostCallback, NULL);

	Detect_InitDevice(DeviceID_IMU, 500, NULL, NULL);

	Detect_InitDevice(DeviceID_PC, 500, Gimbal_VisionLostCallback, Gimbal_VisionRecoverCallback);

	Detect_InitDevice(DeviceID_TOF, 500, NULL, NULL);
}

// ���߼������ص�
void Task_Detect_Callback()
{
	uint32_t presentTime = HAL_GetTick();
	for (uint8_t id = 0; id < DETECT_DEVICE_NUM; id++)
	{
		// �ж��Ƿ����
		if (presentTime - detectList[id].lastRecieveTime > detectList[id].maxInterval && detectList[id].isLost == 0)
		{
			// �ж�ִ��Ĭ�ϻ����Զ���Ĵ�����
			if (detectList[id].lostFunc == NULL)
				Detect_DefaultLostHandler(id);
			else
				detectList[id].lostFunc();
			// ���±�ʶ
			detectList[id].isLost = 1;
		}
		else if (presentTime - detectList[id].lastRecieveTime <= detectList[id].maxInterval && detectList[id].isLost == 1)
		{
			// �ж�ִ��Ĭ�ϻ����Զ���Ĵ�����
			if (detectList[id].recoverFunc == NULL)
				Detect_DefaultRecoverHandler(id);
			else
				detectList[id].recoverFunc();
			// ���±�ʶ
			detectList[id].isLost = 0;
		}
	}
}

// ��ȡ�豸�������
// ����ֵ 1-���� 0-����
uint8_t Detect_IsDeviceLost(uint8_t deviceID)
{
	return detectList[deviceID].isLost;
}

void os_DetectCallback(void const *argument)
{
	osDelay(3000);
	for (;;)
	{
		Task_Detect_Callback();
		osDelay(10);
	}
}
