/**
 * @file Chassis.c
 * @author Screeps
 * @brief  �����ļ�
 * @version 0.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Chassis.h"
#include "arm_math.h"
#include "rc.h"
#include "UserFreertos.h"
#include "gimbal.h"
#include "super_cap.h"
#include "detect.h"
#include "shooter.h"

#define EN_CHASSIS_TASK // ʹ������

// �ڲ����߽ӿ�����
void Chassis_PIDInit(void);
void Chassis_RegisterEvents(void);
char *Chassis_GetModeText(void);
void Chassis_UpdateSpeedLimit(void);
void Chassis_UpdateSlope(void);
void Chassis_RockerCtrl(void);
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Task_Chassis_Callback(void);

Chassis chassis = {0};

/**
 * @brief ���̳�ʼ��
 *
 */
void Chassis_Init()
{
	// ���̳ߴ���Ϣ�����ڽ������٣�
	chassis.info.wheelbase = 466; // �����ʱ��������
	chassis.info.wheeltrack = 461;
	chassis.info.wheelRadius = 76;
	chassis.info.offsetX = 0;
	chassis.info.offsetY = 0;
	// �ƶ�������ʼ��
	chassis.move.maxVx = 2000;
	chassis.move.maxVy = 2000;
	chassis.move.maxVw = 2;
	chassis.speedRto = 1;
	// ��ת������ʼ��
	chassis.rotate.InitAngle = 3000; // ƽ��ʱyaw�����Ƕ�  yaw���ȶ� ����Ҫ��
	// б�º�����ʼ��
	Slope_Init(&chassis.move.xSlope, 5, 5);
	Slope_Init(&chassis.move.ySlope, 5, 5);
	// ���pid��ʼ��
	Chassis_PIDInit();
	// ע���¼�
	Chassis_RegisterEvents();
}
/**
 * @brief ��ʼ��chassis UI��״
 *
 * @attention �ڷ���ʱ�뱣֤����ϵͳ�����ѿ��������ɹ�����
 */
void Chassis_UI_Init()
{
	Graph_SetText(&chassis.ui.mode, "CHA", Color_Orange, 2, 0, 100, 800, "Chassis Mode = Follow", 21, 20);
	Graph_DrawText(&chassis.ui.mode, Operation_Add);
	Graph_SetText(&chassis.ui.speed, "SPD", Color_Orange, 2, 0, 100, 830, "Chassis Speed = Fast", 20, 20);
	Graph_DrawText(&chassis.ui.speed, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT1", Color_Yellow, 2, 0, 1100, 205, 1360, 205); // �����ߣ���ͷˮƽ��
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT2", Color_Yellow, 2, 0, 560, 205, 820, 205);
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT3", Color_Yellow, 2, 0, 1360, 205, 1420, 175);
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT4", Color_Yellow, 2, 0, 480, 155, 560, 205);
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);

	Graph_SetText(&chassis.ui.super_cap, "CAP", Color_Orange, 2, 0, 100, 650, "CAP ENERGY= 100 %", 17, 20);
	Graph_DrawText(&chassis.ui.super_cap, Operation_Add);
}

/************�ڲ����ߺ���*****************/
/**
 * @brief ����pid������ʼ��
 *
 */
void Chassis_PIDInit()
{
	// �ٶȻ�
	PID_Init(&chassis.motors[0].speedPID, 20, 0, 0, 8000, 10000);
	PID_Init(&chassis.motors[1].speedPID, 20, 0, 0, 8000, 10000);
	PID_Init(&chassis.motors[2].speedPID, 20, 0, 0, 8000, 10000);
	PID_Init(&chassis.motors[3].speedPID, 20, 0, 0, 8000, 10000);

	PID_Init(&chassis.rotate.pid, 0.04, 0, 0, 0.5, 0.5);
	PID_SetDeadzone(&chassis.rotate.pid, 2);
}

/**
 * @brief ע�������¼�
 *
 */
void Chassis_RegisterEvents()
{
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnDown, Chassis_Move_KeyCallback); // WASD���£������ƶ�
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnUp, Chassis_Stop_KeyCallback);	// WASD�ɿ�������ֹͣ�ƶ�
	RC_Register(Key_Q | Key_E | Key_R, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchMode_KeyCallback);	// QER�л�����ģʽ
	RC_Register(Key_C, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchSpeed_KeyCallback);					// ����c�¶�
}
/**
 * @brief ����������趨�ƶ��ٶ�
 *
 */
void Chassis_UpdateSpeedLimit()
{
	uint8_t maxPower = JUDGE_GetChassisPowerLimit(); // ��ȡ���̹�������
	float ratio = (maxPower - 50) / (120 - 50);
	if (JUDGE_IsValid() != true) // ������Ч��δ��װ����ϵͳ
		ratio = 1;
	chassis.move.maxVx = (1900 + 1600 * ratio);
	chassis.move.maxVy = (1900 + 1600 * ratio);
	chassis.move.maxVw = (2.375f + 0.8f * ratio);
	Slope_SetStep(&chassis.move.xSlope, 4.75f + 4 * ratio);
	Slope_SetStep(&chassis.move.ySlope, 4.75f + 4 * ratio);
	PID_SetMaxOutput(&chassis.rotate.pid, chassis.move.maxVw); // �趨�����ת�ٶ�
}

/**
 * @brief ����б�¼����ٶ�
 *
 */
void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
}
/**
 * @brief ҡ�˿��Ƶ���
 *
 */
void Chassis_RockerCtrl()
{
	// �󲦸˲����·�����С���ݣ��������������ģʽ
	if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left != 2)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown); // ģ������ְ��°���
	else if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown);
	// �趨�ƶ������ٶ�
	Slope_SetTarget(&chassis.move.xSlope, (float)rcInfo.ch3 * chassis.move.maxVx / 660); // �趨б��Ŀ��ֵ
	Slope_SetTarget(&chassis.move.ySlope, (float)rcInfo.ch4 * chassis.move.maxVy / 660);
}
/**
 * @brief ��ȡ����ģʽUI����
 *
 * @return char* ����ģʽ��ʾ��Ϣ
 */
char *Chassis_GetModeText()
{
	switch (chassis.rotate.mode)
	{
	case ChassisMode_Spin:
		return "Chassis Mode = Spin  ";
	case ChassisMode_Follow:
		return "Chassis Mode = Follow";
	case ChassisMode_Snipe_10:
		return "Chassis Mode = Snipe1";
	case ChassisMode_Snipe_20:
		return "Chassis Mode = Snipe2";
	default:
		return NULL;
	}
}
/************�ⲿ�ӿں���*******************/

/**
 * @brief ������������Ʊ���(������������ʱ<1)
 *
 * @return float ������ưٷֱ�
 * @attention ���̻�������ֵ>30����²����Ƶ�����
 */
float Chassis_GetOutputRatio()
{
	uint16_t powerBuf = JUDGE_GetPowerBuffer(); // ��ȡ��������ֵ

	if (JUDGE_IsValid() != true) // ����ϵͳ������Ч
		powerBuf = 40;

	if (powerBuf > 40) // ������30�����Ƶ�����
	{
		return 1;
	}
	else
	{
		return (powerBuf / 40.0f) * (powerBuf / 40.0f); // ��������С��30�򰴱������Ƶ�����
	}
}
/*************************RC�¼�**************************
���������ܼ���event����
*********************************************************/
// WASD�ƶ�
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
		Slope_SetTarget(&chassis.move.ySlope, -chassis.move.maxVy * chassis.speedRto); // �趨б��Ŀ��ֵ
		break;
	case Key_S:
		Slope_SetTarget(&chassis.move.ySlope, chassis.move.maxVy * chassis.speedRto);
		break;
	case Key_D:
		Slope_SetTarget(&chassis.move.xSlope, -chassis.move.maxVx * chassis.speedRto);
		break;
	case Key_A:
		Slope_SetTarget(&chassis.move.xSlope, chassis.move.maxVx * chassis.speedRto);
		break;
	default:
		break;
	}
}

// ֹͣ�ƶ������ص�
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
	case Key_S:
		Slope_SetTarget(&chassis.move.ySlope, 0); // �趨б��Ŀ��ֵΪ0
		break;
	case Key_A:
	case Key_D:
		Slope_SetTarget(&chassis.move.xSlope, 0);
		break;
	default:
		break;
	}
}
// ��/�����ƶ������ص�
void Chassis_SwitchSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (chassis.speedRto == 1)
		chassis.speedRto = 0.5;
	else
		chassis.speedRto = 1;
	Graph_SetText(&chassis.ui.speed, "SPD", Color_Orange, 2, 0, 100, 830, chassis.speedRto == 1 ? "Chassis Speed = Fast" : "Chassis Speed = Slow", 20, 20);
	Graph_DrawText(&chassis.ui.speed, Operation_Change);
}

// �л�����ģʽ
void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	float H;
	float V;
	float T;

	if (chassis.rotate.mode != ChassisMode_Follow) //  Q/E/R�л�ģʽ �ٴΰ��»ص�����ģʽ
	{
		chassis.rotate.mode = ChassisMode_Follow;
		chassis.speedRto = 1;
	}
	else
	{
		switch (key)
		{
		case Key_Q: // С����ģʽ
			PID_Clear(&chassis.rotate.pid);
			chassis.rotate.mode = ChassisMode_Spin;
			break;
		case Key_E: // �ѻ�ģʽ10m
			chassis.rotate.mode = ChassisMode_Snipe_10;
			if (shooter.distance > 1.5f)
			{
				H = 0.2f;
				V = 15.5f;
				T = 0.020825f * pow(V, 2) - 0.204082f * H - 0.004165f * sqrt(25 * pow(V, 4) - 490.0f * H * pow(V, 2) - 2401.0f * pow(shooter.distance, 2));
				T = sqrt(T);
				gimbal.pitch.targetAngle = acos(shooter.distance / (V * T)) * 180.0f / PI;
			}
			else
			{
				gimbal.pitch.targetAngle = 14.7f;
			}
			LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
			chassis.speedRto = 0.1; // ����
			break;
		case Key_R: // �ѻ�ģʽ20m
			chassis.rotate.mode = ChassisMode_Snipe_20;
			gimbal.pitch.targetAngle = (-36); // shooter.distance  //
			chassis.speedRto = 0.1;			  // ����
		default:
			break;
		}
	}
	Graph_SetText(&chassis.ui.mode, "CHA", Color_Orange, 2, 0, 100, 800, Chassis_GetModeText(), 21, 20);
	Graph_DrawText(&chassis.ui.mode, Operation_Change);
}

/************************freertos����**********************
����������freertos����ϵͳ����
**********************************************************/

// ��������ص�����
void Task_Chassis_Callback()
{
	Chassis_UpdateSpeedLimit();
	chassis.rotate.nowAngle = gimbal.yawMotor.angle;
	chassis.rotate.relativeAngle = MOTOR_M6020_CODE2DGR(chassis.rotate.nowAngle - chassis.rotate.InitAngle);

	if (rcInfo.wheel > 600) // ң���������л�ҡ�˿���/���̿���
		chassis.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		chassis.rockerCtrl = false;

	if (chassis.rockerCtrl) // ҡ�˿���
		Chassis_RockerCtrl();

	/***�������ƽ���ٶ�****/

	Chassis_UpdateSlope(); // �����˶�б�º�������

	// ����̨����ϵ��ƽ���ٶȽ��㵽����ƽ���ٶ�(������̨ƫ���)
	float gimbalAngleSin = arm_sin_f32(chassis.rotate.relativeAngle * PI / 180);
	float gimbalAngleCos = arm_cos_f32(chassis.rotate.relativeAngle * PI / 180);
	chassis.move.vx = Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin;
	chassis.move.vy = -Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos;

	/***���������ת�ٶ�***/

	if (chassis.rotate.mode == ChassisMode_Follow) // ����ģʽ
	{
		PID_SingleCalc(&chassis.rotate.pid, 0, chassis.rotate.relativeAngle);
		chassis.move.vw = chassis.rotate.pid.output;
	}
	else if (chassis.rotate.mode == ChassisMode_Spin) // С����ģʽ
	{
		float ratio = 1 - (ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy) / 2;
		chassis.move.vw = chassis.move.maxVw * ratio * chassis.speedRto;
	}
	else if (chassis.rotate.mode == ChassisMode_Snipe_10 || chassis.rotate.mode == ChassisMode_Snipe_20) // �ѻ�ģʽ
	{
		PID_SingleCalc(&chassis.rotate.pid, 45, chassis.rotate.relativeAngle); // ���̺���  ĳ����45����ܺ�����������û����
		chassis.move.vw = chassis.rotate.pid.output;
	}
	/***���������ת��****/
	float rotateRatio[4];
	rotateRatio[0] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f - chassis.info.offsetY + chassis.info.offsetX;
	rotateRatio[1] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f - chassis.info.offsetY - chassis.info.offsetX;
	rotateRatio[2] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f + chassis.info.offsetY + chassis.info.offsetX;
	rotateRatio[3] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f + chassis.info.offsetY - chassis.info.offsetX;
	int16_t wheelRPM[4];
	wheelRPM[0] = (chassis.move.vx + chassis.move.vy - chassis.move.vw * rotateRatio[0]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19;	// FL
	wheelRPM[1] = -(-chassis.move.vx + chassis.move.vy + chassis.move.vw * rotateRatio[1]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19; // FR
	wheelRPM[2] = (-chassis.move.vx + chassis.move.vy - chassis.move.vw * rotateRatio[2]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19;	// BL
	wheelRPM[3] = -(chassis.move.vx + chassis.move.vy + chassis.move.vw * rotateRatio[3]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19;	// BR
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis.motors[i].targetSpeed = wheelRPM[i];
	}
}

#ifdef EN_CHASSIS_TASK
void os_ChassisCallback(void const *argument)
{

	for (;;)
	{
		Task_Chassis_Callback();
		osDelay(2);
	}
}
#endif
