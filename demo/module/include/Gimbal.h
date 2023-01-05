#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "Moto.h"
#include "Filter.h"
#include "stdbool.h"
#include "user_imu.h"
#include "slope.h"
#include "graphics.h"


typedef struct{
  struct
  {
    float  initAngle;           //yaw
    float  angle,lastAngle,totalAngle,totalRound; //用于角度统计
    float  gyro;
	  float  targetAngle;
    CascadePID  imuPID;  //yaw陀螺仪pid
  }yaw;
  
  struct
  {
    float  initAngle;  
    float  angle;             //pitch
    float  gyro;     
    float  targetAngle;      
	  float  pitchMax,pitchMin;//限幅
    CascadePID imuPID;  //pitch陀螺仪pid
  }pitch;
  
  SingleMotor  yawMotor,pitchMotor;  //yaw pitch电机
  
 
  
	singlePID pitchVisionPID,yawVisionPID;//视觉控制PID
	bool visionEnable;//标记自瞄是否开启
	bool rockerCtrl;//标记是否为摇杆控制，若为false则由鼠标控制
	int16_t visionPitchIncLevel;//视觉pitch角度增益等级
	float manualYawOffset;//手动调整的yaw云台偏移值
  
  struct{
  Circle visionState;//中央圆圈显示自瞄状态
  Text pitchIncLevel;//视觉pitch角度增益等级
	}ui;//自定义UI图形
    
  struct{
    AverFilter pitchFilter,yawFilter; //鼠标数据均值滤波器
    float pitchDPI,yawDPI;       //调整鼠标DPI
  }Mouse;
  
	struct{
		AverFilter pitch,yaw,find;
	}visionFilter;//视觉数据均值滤波器
}Gimbal;

extern Gimbal gimbal;

void Gimbal_Init(void);
void Gimbal_UI_Init(void);
void Gimbal_VisionLostCallback(void);
void Gimbal_VisionRecoverCallback(void);

#endif
