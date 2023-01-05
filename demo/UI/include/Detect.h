#ifndef _DETECT_H_
#define _DETECT_H_

#include "main.h"

//�����������ϵ��豸�ı�ʶ(���豸������Ϣ�б��е��±�)
enum
{
	//���̵��
	DeviceID_ChassisMotor1=0,
	DeviceID_ChassisMotor2,
	DeviceID_ChassisMotor3,
	DeviceID_ChassisMotor4,
	//��̨���
	DeviceID_YawMotor,
	DeviceID_PitchMotor,
	//Ħ���ֵ��
	DeviceID_FricMotor1,
	DeviceID_FricMotor2,
	//�������
	DeviceID_TrigMotor,
	//ң�ؽ�����
	DeviceID_RC,
	//����ϵͳ����
	DeviceID_Judge,
	//������
	DeviceID_IMU,
	//�Ӿ�MINI PC
	DeviceID_PC,
  //���TOF
	DeviceID_TOF,
	DETECT_DEVICE_NUM//������󣬴����豸����(�豸������Ϣ�б���)
};

//���߼��ṹ�壬ÿ���豸��Ӧһ���ṹ�����
typedef struct _DetectDevice
{
	uint32_t maxInterval;//�յ����ݵ������ʱ�䣬��������Ϊ�Ͽ�
	uint32_t lastRecieveTime;//��¼�����ϴ��յ����ݵ�ʱ��
	uint8_t isLost;//����Ƿ��Ѷ�ʧ
	void (*lostFunc)(void);//���ߴ�����
	void (*recoverFunc)(void);//�ָ����Ӵ�����
}DetectDevice;

/****�ӿں�������****/
//�������ݽ���ʱ��
void Detect_Update(uint8_t deviceID);
//��ȡ�豸��ʧ���
uint8_t Detect_IsDeviceLost(uint8_t deviceID);
//��ʼ��
void Detect_InitAll(void);
//����ص�
void Task_Detect_Callback(void);

#endif
