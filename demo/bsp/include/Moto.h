#ifndef _MOTO_H_
#define _MOTO_H_
#include "stdint.h"
#include "PID.h"

// ���ֵ������ֵ��ǶȵĻ���
#define MOTOR_M3508_DGR2CODE(dgr) ((int32_t)((dgr)*436.9263f)) // 3591/187*8191/360
#define MOTOR_M3508_CODE2DGR(code) ((float)((code) / 436.9263f))

#define MOTOR_M2006_DGR2CODE(dgr) ((int32_t)((dgr)*819.1f)) // 36*8191/360
#define MOTOR_M2006_CODE2DGR(code) ((float)((code) / 819.1f))

#define MOTOR_M6020_DGR2CODE(dgr) ((int32_t)((dgr)*22.7528f)) // 8191/360
#define MOTOR_M6020_CODE2DGR(code) ((float)((code) / 22.7528f))

#define CHASSIS_MOTORS_NUM 4
#define FRIC_MOTORS_NUM 2
typedef struct _MOTOR
{
	int16_t angle, speed, torque;
	int8_t temp;

	int16_t lastAngle; // ��¼��һ�εõ��ĽǶ�

	int16_t targetSpeed; // Ŀ���ٶ�
	int32_t targetAngle; // Ŀ��Ƕ�(������ֵ)

	int32_t totalAngle; // �ۼ�ת���ı�����ֵ

	singlePID speedPID;	 // �ٶ�pid(����)
	CascadePID anglePID; // �Ƕ�pid������
} SingleMotor;

// ��ʼ�������ۼƽǶ�
void Motor_StartCalcAngle(SingleMotor *motor);
// �������ۼ�ת����Ȧ��
void Motor_CalcAngle(SingleMotor *motor);
// ���µ������(���ܽ����˲�)
void Motor_Update(SingleMotor *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp);
#endif
