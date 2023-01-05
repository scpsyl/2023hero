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
    float  angle,lastAngle,totalAngle,totalRound; //���ڽǶ�ͳ��
    float  gyro;
	  float  targetAngle;
    CascadePID  imuPID;  //yaw������pid
  }yaw;
  
  struct
  {
    float  initAngle;  
    float  angle;             //pitch
    float  gyro;     
    float  targetAngle;      
	  float  pitchMax,pitchMin;//�޷�
    CascadePID imuPID;  //pitch������pid
  }pitch;
  
  SingleMotor  yawMotor,pitchMotor;  //yaw pitch���
  
 
  
	singlePID pitchVisionPID,yawVisionPID;//�Ӿ�����PID
	bool visionEnable;//��������Ƿ���
	bool rockerCtrl;//����Ƿ�Ϊҡ�˿��ƣ���Ϊfalse����������
	int16_t visionPitchIncLevel;//�Ӿ�pitch�Ƕ�����ȼ�
	float manualYawOffset;//�ֶ�������yaw��̨ƫ��ֵ
  
  struct{
  Circle visionState;//����ԲȦ��ʾ����״̬
  Text pitchIncLevel;//�Ӿ�pitch�Ƕ�����ȼ�
	}ui;//�Զ���UIͼ��
    
  struct{
    AverFilter pitchFilter,yawFilter; //������ݾ�ֵ�˲���
    float pitchDPI,yawDPI;       //�������DPI
  }Mouse;
  
	struct{
		AverFilter pitch,yaw,find;
	}visionFilter;//�Ӿ����ݾ�ֵ�˲���
}Gimbal;

extern Gimbal gimbal;

void Gimbal_Init(void);
void Gimbal_UI_Init(void);
void Gimbal_VisionLostCallback(void);
void Gimbal_VisionRecoverCallback(void);

#endif
