#ifndef _USER_FREERTOS_H_
#define _USER_FREERTOS_H_
#include "cmsis_os.h"


//cube���ɵĹ��ⲿ�ļ����õı���
//task
extern  osThreadId IMUTaskHandle;
extern  osThreadId LEDTaskHandle;
extern  osThreadId MotorTaskHandle;
extern  osThreadId ChassisTaskHandle;
extern  osThreadId RcTaskHandle;
extern  osThreadId ErrorTaskHandle;
extern  osThreadId GimbalTaskHandle;
extern  osThreadId ShooterTaskHandle;
extern  osThreadId VisionTxTaskHandle;
extern  osThreadId BeepTaskHandle;
extern  osThreadId JudgeTaskHandle;
extern  osThreadId DetectTaskHandle;
#endif

