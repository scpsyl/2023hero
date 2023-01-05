#include "Moto.h"
#include "chassis.h"
#include "Gimbal.h"
#include "shooter.h"
#include "USER_CAN.h"
#include "UserFreertos.h"
#include "vision.h"
#define EN_MOTOR_TASK // 使能任务

// 开始计算电机累计角度
void Motor_StartCalcAngle(SingleMotor *motor)
{
	motor->totalAngle = 0;
	motor->lastAngle = motor->angle;
	motor->targetAngle = 0;
}

// 计算电机累计转过的圈数
void Motor_CalcAngle(SingleMotor *motor)
{
	int32_t dAngle = 0;
	if (motor->angle - motor->lastAngle < -4000)
		dAngle = motor->angle + (8191 - motor->lastAngle);
	else if (motor->angle - motor->lastAngle > 4000)
		dAngle = -motor->lastAngle - (8191 - motor->angle);
	else
		dAngle = motor->angle - motor->lastAngle;
	// 将角度增量加入计数器
	motor->totalAngle += dAngle;
	// 记录角度
	motor->lastAngle = motor->angle;
}

// 更新电机数据(可能进行滤波)
void Motor_Update(SingleMotor *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp)
{
	motor->angle = angle;
	motor->speed = speed;
	motor->torque = torque;
	motor->temp = temp;
}

/************************freertos任务****************************/
// 所有CAN电机的控制任务
void Task_CANMotors_Callback()
{
	for (uint8_t i = 0; i < 4; i++)
		PID_SingleCalc(&chassis.motors[i].speedPID, chassis.motors[i].targetSpeed, chassis.motors[i].speed);

	PID_SingleCalc(&shooter.fricMotor[0].speedPID, shooter.fricMotor[0].targetSpeed, shooter.fricMotor[0].speed);
	PID_SingleCalc(&shooter.fricMotor[1].speedPID, shooter.fricMotor[1].targetSpeed, shooter.fricMotor[1].speed);
	Motor_CalcAngle(&shooter.triggerMotor);
	PID_CascadeCalc(&shooter.triggerMotor.anglePID, shooter.triggerMotor.targetAngle, shooter.triggerMotor.totalAngle, shooter.triggerMotor.speed);
	//  pitch轴电机输出由pitchImuPID直接给出
	//	Motor_CalcAngle(&gimbal.pitchMotor);
	//	PID_CascadeCalc(&gimbal.pitchMotor.anglePID,gimbal.pitchMotor.targetAngle,gimbal.pitchMotor.totalAngle,gimbal.pitchMotor.speed);
	//  yaw轴电机输出由yawImuPID直接给出
	//  Motor_CalcAngle(&gimbal.yawMotor);
	//	PID_CascadeCalc(&gimbal.yawMotor.anglePID,gimbal.yawMotor.targetAngle,gimbal.yawMotor.totalAngle,gimbal.yawMotor.speed);

	float chassisOutputRatio = Chassis_GetOutputRatio(); // 用于超功率保护
	USER_CAN_SetMotorCurrent(&hcan1, 0x200,
							 chassis.motors[0].speedPID.output * chassisOutputRatio,
							 chassis.motors[1].speedPID.output * chassisOutputRatio,
							 chassis.motors[2].speedPID.output * chassisOutputRatio,
							 chassis.motors[3].speedPID.output * chassisOutputRatio);
	USER_CAN_SetMotorCurrent(&hcan1, 0x1FF, shooter.triggerMotor.anglePID.output,
							 gimbal.yaw.imuPID.output,
							 0, 0);
	USER_CAN_SetMotorCurrent(&hcan2, 0x200,
							 0,
							 0,
							 shooter.fricMotor[0].speedPID.output,
							 shooter.fricMotor[1].speedPID.output);
	USER_CAN_SetMotorCurrent(&hcan2, 0x1FF, 0, 0,
							 gimbal.pitch.imuPID.output, 0);
}

#ifdef EN_MOTOR_TASK
void os_MotorCallback(void const *argument)
{
	osDelay(500);
	for (;;)
	{
		Task_CANMotors_Callback();
		osDelay(2);
	}
}
#endif
