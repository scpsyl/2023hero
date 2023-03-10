#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#endif

typedef struct _PID
{
	float kp,ki,kd;
	float error,lastError;//误差、上次误差
	float integral,maxIntegral;//积分、积分限幅
	float output,maxOutput;//输出、输出限幅
	float deadzone;//死区
}singlePID;

typedef struct _CascadePID
{
	singlePID inner;//内环
	singlePID outer;//外环
	float output;//串级输出，等于inner.output
}CascadePID;

void PID_Init(singlePID *pid,float p,float i,float d,float maxSum,float maxOut);
void PID_SingleCalc(singlePID *pid,float reference,float feedback);
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void PID_Clear(singlePID *pid);
void PID_SetMaxOutput(singlePID *pid,float maxOut);
void PID_SetDeadzone(singlePID *pid,float deadzone);

#endif
