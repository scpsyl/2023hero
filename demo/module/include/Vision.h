#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "stdbool.h"
//数据帧命令码
typedef enum{
	VisionCmd_EnemyColor=0x01,
	VisionCmd_BloodInfo1,
	VisionCmd_BloodInfo2,
	VisionCmd_ShootSpeed,
	VisionCmd_ModeSelect
}VisionCmd;

//视觉模式
typedef enum{
	VisionMode_Manual=0,
	VisionMode_AutoAim,
	VisionMode_SmallBuff,
	VisionMode_BigBuff,
	VisionMode_AimAt1,
	VisionMode_AimAt2,
	VisionMode_AimAt3,
	VisionMode_AimAt4,
	VisionMode_AimAt5,
	VisionMode_AimAt6,
	VisionMode_AimAt7,
	VisionMode_AimAt8,
}VisionMode;

//视觉数据接收结构体
typedef struct{	
	float pitch;
	float yaw;
	int16_t distance;
	uint8_t find;
}VisionRecv;

//视觉系统状态
typedef struct{
	bool isRunning;
	VisionMode mode;
}VisionState;

extern VisionRecv visionRecv;
extern VisionState visionState;

void Vision_ParseData(uint8_t *buf);
void Vision_DetectFirstRecv(void);
void Vision_SendEnemyColor(uint8_t color);
void Vision_SendBloodInfo1(uint16_t blood1,uint16_t blood2,uint16_t blood3);
void Vision_SendBloodInfo2(uint16_t blood4,uint16_t blood5,uint16_t blood7);
void Vision_SendShootSpeed(float speed);
void Vision_SetMode(VisionMode mode);
void Vision_SendInfo(float motorYaw,float motorPitch,float shootSpeed,uint8_t color,uint8_t enable,uint8_t wind);

#endif
