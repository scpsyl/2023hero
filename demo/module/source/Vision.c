#include "Vision.h"
#include <string.h>
#include "Judge.h"
#include "Beep.h"
#include "gimbal.h"
#include "moto.h"
#include "user_imu.h"
#include "user_can.h"

void Task_VisionSendInfo_Callback(void);

VisionRecv visionRecv;	 // 从视觉收到的数据
VisionState visionState; // 当前视觉状态

void Vision_SendMode(VisionMode mode);

// 解析接收到的数据
void Vision_ParseData(uint8_t *buf)
{
	visionRecv.yaw = ((int16_t)(buf[1] << 8 | buf[2])) * 0.01;
	visionRecv.pitch = ((int16_t)(buf[3] << 8 | buf[4])) * 0.01;
	visionRecv.distance = (buf[5] << 8 | buf[6]);
	visionRecv.find = buf[7] >> 7;
}

// 检测第一次接收到视觉数据，应在接收到数据时调用
void Vision_DetectFirstRecv()
{
	if (!visionState.isRunning)
	{
		visionState.isRunning = true;
		Vision_SendEnemyColor(1 - 0 /*JUDGE_GetSelfColor()*/);
		// for(uint8_t i=0;i<3;i++)
		// Beep_PlayNotes((Note[]){{T_H1,100},{T_None,100}},2);
	}
}

// 发送敌方颜色
void Vision_SendEnemyColor(uint8_t color)
{
	uint8_t buf[3] = {0xAB, VisionCmd_EnemyColor, color};
	USER_CAN_SendVisionBuf(&hcan2, buf, 3);
}

// 发送编号1,2,3机器人的血量数据
void Vision_SendBloodInfo1(uint16_t blood1, uint16_t blood2, uint16_t blood3)
{
	uint8_t buf[8];
	buf[0] = 0xAB;
	buf[1] = VisionCmd_BloodInfo1;
	buf[2] = blood1 >> 8;
	buf[3] = blood1 & 0xFF;
	buf[4] = blood2 >> 8;
	buf[5] = blood2 & 0xFF;
	buf[6] = blood3 >> 8;
	buf[7] = blood3 & 0xFF;
	USER_CAN_SendVisionBuf(&hcan2, buf, 8);
}

// 发送编号4,5,7机器人的血量数据
void Vision_SendBloodInfo2(uint16_t blood4, uint16_t blood5, uint16_t blood7)
{
	uint8_t buf[8];
	buf[0] = 0xAB;
	buf[1] = VisionCmd_BloodInfo2;
	buf[2] = blood4 >> 8;
	buf[3] = blood4 & 0xFF;
	buf[4] = blood5 >> 8;
	buf[5] = blood5 & 0xFF;
	buf[6] = blood7 >> 8;
	buf[7] = blood7 & 0xFF;
	USER_CAN_SendVisionBuf(&hcan2, buf, 8);
}

// 发送射击速度信息
void Vision_SendShootSpeed(float speed)
{
	uint8_t buf[6];
	buf[0] = 0xAB;
	buf[1] = VisionCmd_ShootSpeed;
	memcpy(buf + 2, &speed, 4);
	USER_CAN_SendVisionBuf(&hcan2, buf, 6);
}

// 发送自瞄模式
void Vision_SendMode(VisionMode mode)
{
	uint8_t buf[3];
	buf[0] = 0xAB;
	buf[1] = VisionCmd_ModeSelect;
	buf[2] = mode;
	USER_CAN_SendVisionBuf(&hcan2, buf, 3);
}

// 设置自瞄模式
void Vision_SetMode(VisionMode mode)
{
	Vision_SendMode(mode);
	visionState.mode = mode;
}

// 向视觉发送yaw、pitch、射速、颜色、是否使能 wind=1
void Vision_SendInfo(float motorYaw, float motorPitch, float shootSpeed, uint8_t color, uint8_t enable, uint8_t wind)
{
	uint8_t buf[8];
	buf[0] = 0xAB;
	buf[1] = (int16_t)(motorYaw * 100) >> 8;
	buf[2] = (int16_t)(motorYaw * 100) & 0xff;
	buf[3] = (int16_t)(motorPitch * 100) >> 8;
	buf[4] = (int16_t)(motorPitch * 100) & 0xff;
	buf[5] = (int16_t)(shootSpeed * 100) >> 8;
	buf[6] = (int16_t)(shootSpeed * 100) & 0xff;
	buf[7] = color << 7 | enable << 6 | wind << 4;
	USER_CAN_SendVisionBuf(&hcan2, buf, 8);
}

/**********************freertos任务******************************/
// 向视觉定时发送数据任务回调
void Task_VisionSendInfo_Callback()
{
	float shootSpd = 15.5f;
	if (JUDGE_GetShootSpeedLimit() == 10)
		shootSpd = 9.2f;
	else if (JUDGE_GetShootSpeedLimit() == 16)
		shootSpd = 15.5f;
	Vision_SendInfo(gimbal.yaw.angle,
					gimbal.pitch.angle,
					shootSpd, 1 - JUDGE_GetSelfColor(), gimbal.visionEnable, 1);
}

void os_VisionCallback(void const *argument)
{
	osDelay(1000);
	for (;;)
	{
		Task_VisionSendInfo_Callback();
		osDelay(2);
	}
}
