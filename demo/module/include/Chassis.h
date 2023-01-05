#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "pid.h"
#include "moto.h"
#include "slope.h"
#include <stdbool.h>
#include "judge.h"
#include "graphics.h"

typedef enum _chassisMode
{
	ChassisMode_Follow,	  // ���̸�����̨ģʽ
	ChassisMode_Spin,	  // С����ģʽ
	ChassisMode_Snipe_10, // �ѻ�ģʽ10m����������̨��45�ȼнǣ����ٴ������,���DPS�������
	ChassisMode_Snipe_20  // 20m
} ChassisMode;

typedef struct _Chassis
{
	// ���̳ߴ���Ϣ
	struct Info
	{
		float wheelbase;   // ���
		float wheeltrack;  // �־�
		float wheelRadius; // �ְ뾶
		float offsetX;	   // ������xy���ϵ�ƫ��
		float offsetY;
	} info;
	// 4�����
	SingleMotor motors[4];
	float speedRto; // �����ٶȰٷֱ� ���ڵ����˶�
	// �����ƶ���Ϣ
	struct Move
	{
		float vx; // ��ǰ����ƽ���ٶ� mm/s
		float vy; // ��ǰǰ���ƶ��ٶ� mm/s
		float vw; // ��ǰ��ת�ٶ� rad/s

		float maxVx, maxVy, maxVw; // ������������ٶ�
		Slope xSlope, ySlope;	   // ����������б��
	} move;
	// ��ת�����Ϣ
	struct
	{
		singlePID pid;			 // ��תPID����relativeAngle���������ת�ٶ�
		float relativeAngle; // ��̨����̵�ƫ��� ��λ��
		int16_t InitAngle;	 // ��̨����̶���ʱ�ı�����ֵ
		int16_t nowAngle;	 // ��ʱ��̨�ı�����ֵ
		ChassisMode mode;	 // ����ģʽ
	} rotate;

	// �Զ���UI��״
	struct
	{
		Text mode;		// ��ʾ����ģʽ
		Text speed;		// ��ʾ�Ƿ��¶ף����٣�
		Line outline;	// ��ʾ��������
		Text super_cap; // ��ʾ����

	} ui;

	bool rockerCtrl;

} Chassis;

extern Chassis chassis;

void Chassis_Init(void);
void Chassis_UI_Init(void);
float Chassis_GetOutputRatio(void);
#endif
