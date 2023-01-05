#ifndef _MOTO_H_
#define _MOTO_H_
#include "stdint.h"
#include "PID.h"

// 各种电机编码值与角度的换算
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

	int16_t lastAngle; // 记录上一次得到的角度

	int16_t targetSpeed; // 目标速度
	int32_t targetAngle; // 目标角度(编码器值)

	int32_t totalAngle; // 累计转过的编码器值

	singlePID speedPID;	 // 速度pid(单级)
	CascadePID anglePID; // 角度pid，串级
} SingleMotor;

// 开始计算电机累计角度
void Motor_StartCalcAngle(SingleMotor *motor);
// 计算电机累计转过的圈数
void Motor_CalcAngle(SingleMotor *motor);
// 更新电机数据(可能进行滤波)
void Motor_Update(SingleMotor *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp);
#endif
