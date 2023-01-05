#include "Gimbal.h"
#include "RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Detect.h"
#include <stdio.h>
#include "UserFreertos.h"
#include "beep.h"
#define EN_GIMBAL_TASK // ʹ������

void Gimbal_InitPID(void);
void Gimbal_RegisterEvents(void);
void Gimbal_UpdataAngle(void);
void Gimbal_MouseCtrl(void);
void Gimbal_VisionCtrl(void);
void Gimbal_RockerCtrl(void);
void Gimbal_SwitchVision_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_SelectVisionTarget_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_ChangeVisionPitchInc_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_ManualAdjustYaw_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_CalibrationUI_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

Gimbal gimbal = {0};
float temp;
/********************��ʼ��************************/
// ��ʼ����̨
void Gimbal_Init()
{

	gimbal.pitch.pitchMax = 42; // �趨pitch�Ƕ��޷�
	gimbal.pitch.pitchMin = -19;

	gimbal.pitch.initAngle = 0; // ������pitch�����Ƕ� *0��ʾpitchˮƽ
	gimbal.pitch.targetAngle = gimbal.pitch.initAngle;

	gimbal.yaw.initAngle = 0; // ������yaw�����Ƕ�   *0��ʾyaw����
	gimbal.yaw.targetAngle = gimbal.yaw.initAngle;

	gimbal.visionEnable = false; // ����Ĭ�Ϲر�

	gimbal.rockerCtrl = false; // Ĭ��ʹ�ü��̿���

	gimbal.visionPitchIncLevel = 0; // pitch�����ֵ
	gimbal.manualYawOffset = 0;		// yawƫ��ֵ

	Filter_InitAverFilter(&gimbal.Mouse.yawFilter, 10); // �������ݾ�ֵ�˲� ƽ���������
	Filter_InitAverFilter(&gimbal.Mouse.pitchFilter, 10);

	Filter_InitAverFilter(&gimbal.visionFilter.find, 50); // �Ӿ����ݾ�ֵ�˲�

	Gimbal_InitPID();		 // ��ʼ��PID����
	Gimbal_RegisterEvents(); // ע���¼�
}

// ��ʼ��gimbal UI��״
void Gimbal_UI_Init()
{
	Graph_SetCircle(&gimbal.ui.visionState, "VSN", Color_White, 5, 2, 960, 540, 30);
	Graph_SetText(&gimbal.ui.pitchIncLevel, "PTH", Color_Orange, 2, 3, 100, 770, "Pitch Level = 000", 17, 20);
	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Add);
	Graph_DrawText(&gimbal.ui.pitchIncLevel, Operation_Add);
}

/************�ڲ����ߺ���*****************/

// ��ʼ��PID����
void Gimbal_InitPID()
{
	/*pitch�������ǿ���*/
	PID_Init(&gimbal.pitch.imuPID.inner, 25, 0, 200, 0, 20000);
	PID_Init(&gimbal.pitch.imuPID.outer, -500, 0, 0, 500, 20000);

	/*yaw�������ǿ���*/
	PID_Init(&gimbal.yaw.imuPID.inner, 30, 0, 200, 0, 20000);
	PID_Init(&gimbal.yaw.imuPID.outer, -600, 0, 0, 500, 20000);

	PID_Init(&gimbal.pitchVisionPID, 0.012, 0, 0, 0.5, 0.5);
	PID_Init(&gimbal.yawVisionPID, 0.021, 0, 0.1, 0.5, 0.5);
	//  PID_Init(&gimbal.yawVisionPID,0,0,0,0.5,0.5);
}

// ע���¼�
void Gimbal_RegisterEvents()
{
	RC_Register(Key_Right, CombineKey_None, KeyEvent_OnDown, Gimbal_SwitchVision_KeyCallback); // ����Ҽ���������
	RC_Register(Key_Right, CombineKey_None, KeyEvent_OnUp, Gimbal_SwitchVision_KeyCallback);
	RC_Register(Key_Z | Key_X | Key_C, CombineKey_Shift, KeyEvent_OnDown, Gimbal_SelectVisionTarget_KeyCallback); // shift+z\x\cѡ������Ŀ��
	RC_Register(Key_Q | Key_A, CombineKey_Shift, KeyEvent_OnDown, Gimbal_ChangeVisionPitchInc_KeyCallback);		  // shift+q\a����pitch����
	RC_Register(Key_Z | Key_X, CombineKey_Shift, KeyEvent_OnPressing, Gimbal_ManualAdjustYaw_KeyCallback);		  // ��סshift+z\x����yawƫ��
	RC_Register(Key_R, CombineKey_Ctrl, KeyEvent_OnDown, Gimbal_Return_KeyCallback);							  // ctrl R һ����ͷ
}

// ���������ǽǶȲ��ۻ�yaw�Ƕ�
void Gimbal_UpdataAngle()
{
	gimbal.yaw.gyro = User_Imu_Get_Z_Gyro();
	gimbal.yaw.angle = User_Imu_GetYaw();
	/*c�尲װ������ٷ�imu���ᶨ�巽��ͬ yawһ�� pitch roll�෴*/
	gimbal.pitch.gyro = User_Imu_Get_X_Gyro();
	gimbal.pitch.angle = User_Imu_GetRoll();

	float dAngle = 0;
	if (gimbal.yaw.angle - gimbal.yaw.lastAngle < -270)
		dAngle = gimbal.yaw.angle + (360 - gimbal.yaw.lastAngle);
	else if (gimbal.yaw.angle - gimbal.yaw.lastAngle > 270)
		dAngle = -gimbal.yaw.lastAngle - (360 - gimbal.yaw.angle);
	else
		dAngle = gimbal.yaw.angle - gimbal.yaw.lastAngle;
	gimbal.yaw.totalAngle += dAngle;
	gimbal.yaw.lastAngle = gimbal.yaw.angle;
}

// ��������̨
void Gimbal_MouseCtrl()
{
	if (chassis.rotate.mode == ChassisMode_Snipe_10 || chassis.rotate.mode == ChassisMode_Snipe_20)
	{
		gimbal.Mouse.yawDPI = 0.001 * 0.1;
		gimbal.Mouse.pitchDPI = 0.0005 * 0.1;
	}
	else
	{
		gimbal.Mouse.yawDPI = 0.001;
		gimbal.Mouse.pitchDPI = 0.0005;
	}
	gimbal.yaw.targetAngle -= Filter_AverCalc(&gimbal.Mouse.yawFilter, rcInfo.mouse.x * gimbal.Mouse.yawDPI);		// yaw�޸�
	gimbal.pitch.targetAngle -= Filter_AverCalc(&gimbal.Mouse.pitchFilter, rcInfo.mouse.y * gimbal.Mouse.pitchDPI); // pitch�޸�
	LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);									// pitch�޷�
}

// �Ӿ�������̨
void Gimbal_VisionCtrl()
{
	PID_SingleCalc(&gimbal.yawVisionPID, gimbal.manualYawOffset, visionRecv.yaw);
	gimbal.yaw.targetAngle -= gimbal.yawVisionPID.output;
	PID_SingleCalc(&gimbal.pitchVisionPID, gimbal.visionPitchIncLevel, visionRecv.pitch);
	gimbal.pitch.targetAngle -= gimbal.pitchVisionPID.output; // ��ת��̨pitch
	LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}

// ҡ�˿�����̨
void Gimbal_RockerCtrl()
{
	gimbal.yaw.targetAngle -= rcInfo.ch1 * 0.35 / 660.0f;	// yaw
	gimbal.pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // ��ת��̨pitch
	LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}

/************�ⲿ�ӿں���*******************/
// �Ӿ�PC���߻ص�
void Gimbal_VisionLostCallback()
{
	gimbal.ui.visionState.color = Color_White;
	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
	visionRecv.pitch = visionRecv.yaw = visionRecv.distance = 0;
	visionRecv.find = 0;
}

// �Ӿ��ָ����ӻص�
void Gimbal_VisionRecoverCallback()
{
	gimbal.ui.visionState.color = Color_Yellow;
	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
}

/*************************RC�¼�**************************
���������ܼ���event����
*********************************************************/

// ����������̻ص�
void Gimbal_SwitchVision_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (Detect_IsDeviceLost(DeviceID_PC))
		return;
	switch (event)
	{
	case KeyEvent_OnDown:
		gimbal.visionEnable = true;
		break;
	case KeyEvent_OnUp:
		gimbal.visionEnable = false;
		break;
	default:
		break;
	}
}

// �Ӽ��Ӿ�PITCH����
void Gimbal_ChangeVisionPitchInc_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (key == Key_Q)
		gimbal.visionPitchIncLevel++;
	else if (key == Key_A)
		gimbal.visionPitchIncLevel--;

	char textBuf[30] = {0};

	sprintf(textBuf, "Pitch Level = %03d", gimbal.visionPitchIncLevel);
	Graph_SetText(&gimbal.ui.pitchIncLevel, "PTH", Color_Orange, 2, 0, 100, 770, textBuf, 17, 20);
	Graph_DrawText(&gimbal.ui.pitchIncLevel, Operation_Change);
}

// ѡ������Ŀ��
void Gimbal_SelectVisionTarget_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_Z:
		Vision_SetMode(VisionMode_AimAt1);
		break;
	case Key_X:
		Vision_SetMode(VisionMode_AimAt3);
		break;
	case Key_C:
		Vision_SetMode(VisionMode_AimAt6);
		break;
	default:
		break;
	}
}

// �ֶ�����yaw�Ƕ�ƫ��
void Gimbal_ManualAdjustYaw_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (key == Key_Z)
		gimbal.manualYawOffset -= 1.0f;
	else if (key == Key_X)
		gimbal.manualYawOffset += 1.0f;
}

// һ����ͷ
void Gimbal_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	Slope_SetTarget(&chassis.move.ySlope, 0);
	Slope_SetTarget(&chassis.move.xSlope, 0); // ɲ��
	gimbal.yaw.targetAngle += 180;
}

/************************freertos����**********************
����������freertos����ϵͳ����
**********************************************************/
// ��̨��������
void Task_Gimbal_Callback()
{
	if (rcInfo.wheel > 600) // ң���������л�ҡ�˿��ƻ���������
		gimbal.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		gimbal.rockerCtrl = false;

	if (rcInfo.wheel > 600) // ң����ģʽ�²��ֲ����׶˿�������
		gimbal.visionEnable = true;
	else if (rcInfo.wheel < 600 && gimbal.rockerCtrl) // ң����ģʽ�²��ֲ��ڵ׶˹ر�����
		gimbal.visionEnable = false;

	// ��ʶ��״̬�����˲�����ֹż������ʶ��
	float visionFindAver = Filter_AverCalc(&gimbal.visionFilter.find, visionRecv.find);

	// ����ʶ��״̬����UI
	if (gimbal.ui.visionState.color != Color_Green && visionFindAver > 0.5f)
	{
		gimbal.ui.visionState.color = Color_Green; // ��ɫ����Ȧ��ʾ��ʶ��
		Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
	}
	else if (gimbal.ui.visionState.color != Color_Yellow && visionFindAver < 0.5f)
	{
		gimbal.ui.visionState.color = Color_Yellow; // ��ɫ����Ȧ��ʾδʶ��
		Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
	}

	if (gimbal.visionEnable && visionFindAver >= 0.5f) // ���鿪����ʶ��ɹ�
	{
		Gimbal_VisionCtrl(); // ���Ӿ�������̨
	}
	else // ����δ������δʶ�𵽣���ҡ�˻�������
	{
		if (gimbal.rockerCtrl)
			Gimbal_RockerCtrl();
		else
			Gimbal_MouseCtrl();
	}

	// ���������ǽǶ�
	Gimbal_UpdataAngle();
	// ����yaw������
	PID_CascadeCalc(&gimbal.yaw.imuPID, gimbal.yaw.targetAngle, gimbal.yaw.totalAngle, gimbal.yaw.gyro);
	// ����pitch�����
	PID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
	temp = gimbal.pitchMotor.angle / 100.0;
}

#ifdef EN_GIMBAL_TASK
void os_GimbalCallback(void const *argument)
{

	for (;;)
	{
		Task_Gimbal_Callback();
		osDelay(2);
	}
}
#endif
