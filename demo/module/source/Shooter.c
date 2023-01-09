#include "Shooter.h"
#include "RC.h"
#include <stdio.h>
#include "Userfreertos.h"
#include "judge.h"
//#include "usb_device.h"
#include "beep.h"
#include "user_tof.h"
#define EN_SHOOTER_TASK // ʹ������

Shooter shooter;

void Shooter_InitPID(void);
void Shooter_RegisterEvents(void);
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_ChangeLineIncHigh_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_Distance_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_InquireBulletState(void);
// ���ϵͳ��ʼ��
void Shooter_Init()
{

  shooter.rockerCtrl = false; // ��ʼ״̬�������

  shooter.fricSpd = 5450; // Ĭ��Ħ�����ٶ�5450rpm
	shooter.n20spd = 1000;//���Բ���

  shooter.bulletReadyFlag = true; // ����Ĭ�Ͼ����������޴������������
  shooter.fricOpenFlag = false;   // Ħ����Ĭ�Ͽ���

  Shooter_InitPID();        // ��ʼ�����pid
  Shooter_RegisterEvents(); // ע���¼�
}
// ��ʼ��shooter UI��״
/*�ڷ���ʱ�뱣֤����ϵͳ�����ѿ��������ɹ�����*/
void Shooter_UI_Init()
{
  /**************************��������� Լ 5.71�����/mm**************************************/
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL1", Color_Orange, 2, 1, 890, 425, 1030, 425); // 5m
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  // 10�����һ���ǰ��ս�ߴ�
  Graph_SetLine(&shooter.ui.auxiliaryLine, "095", Color_Orange, 2, 1, 944, 338 + 4, 976, 338 + 4); // 9.5m��
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL2", Color_Pink, 2, 1, 940, 338, 980, 338); // 10m��
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "105", Color_Orange, 2, 1, 944, 338 - 16, 976, 338 - 16); // 10.5m��
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL0", Color_Orange, 2, 1, 865, 540, 1045, 540); // ������
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL5", Color_Orange, 1, 1, 960, 200, 960, 583); // ����
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);

  Graph_SetText(&shooter.ui.auxLineHigh, "ALH", Color_Orange, 2, 2, 100, 740, "Line High = 000", 15, 20);
  Graph_DrawText(&shooter.ui.auxLineHigh, Operation_Add);
  Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric Closed!!!", 14, 20);
  Graph_DrawText(&shooter.ui.fricClose, Operation_Add);
  Graph_SetText(&shooter.ui.TofDistance, "TOF", Color_Green, 1, 2, 1060, 560, "DISTANCE = 00.000", 17, 20);
  Graph_DrawText(&shooter.ui.TofDistance, Operation_Add);
  Graph_SetText(&shooter.ui.blockInfo, "TBK", Color_Yellow, 2, 2, 100, 710, "Trig Blocked", 12, 20);
}
/************�ڲ����ߺ���*****************/
// ��ʼ��PID����
void Shooter_InitPID()
{
  Motor_StartCalcAngle(&shooter.triggerMotor); // ��ʼ������Ƕ��ۼ�
  PID_Init(&shooter.triggerMotor.anglePID.inner, 15, 0, 0, 7800, 20000);
  PID_Init(&shooter.triggerMotor.anglePID.outer, 0.3, 0, 0, 0, 7000);
  PID_Init(&shooter.fricMotor[0].speedPID, 60, 0, 0, 10000, 20000);
  PID_Init(&shooter.fricMotor[1].speedPID, 60, 0, 0, 10000, 20000);
	PID_Init(&shooter.n20Motor.speedPID,200,0.055,0,10000,10000);
}
// ע���¼�
void Shooter_RegisterEvents()
{
  // �������̧�𿪹ز���
  RC_Register(Key_Left, CombineKey_None, KeyEvent_OnDown, Shooter_SwitchState_KeyCallback);
  RC_Register(Key_Left, CombineKey_None, KeyEvent_OnUp, Shooter_SwitchState_KeyCallback);
  RC_Register(Key_Left, CombineKey_None, KeyEvent_OnPressing, Shooter_SwitchState_KeyCallback);
  // F����Ħ����
  RC_Register(Key_F, CombineKey_None, KeyEvent_OnDown, Shooter_StartFric_KeyCallback);
  // shift+E/D����10m�����߸߶�
  RC_Register(Key_E | Key_D, CombineKey_Shift, KeyEvent_OnDown, Shooter_ChangeLineIncHigh_KeyCallback);
  // ��x��ʾ����
  RC_Register(Key_X, CombineKey_None, KeyEvent_OnDown, Shooter_Distance_KeyCallback);
}

// ���ݲ���ϵͳ��Ϣ����Ħ����ת��
void Shooter_UpdateFricSpeed()
{
  if (JUDGE_GetShootSpeedLimit() >= 16)
    shooter.fricSpd = 5400; // 15.5m/s
  else
    shooter.fricSpd = 3800; // 9.2m/s
}

// ң�п���
void Shooter_RockerCtrl()
{
  if (rcInfo.right == 1) // �Ҳ����������Ħ����
  {
    shooter.fricOpenFlag = 1;
    shooter.fricMotor[0].targetSpeed = -(shooter.fricSpd);
    shooter.fricMotor[1].targetSpeed = shooter.fricSpd;
	  shooter.n20Motor.targetSpeed = shooter.n20spd;

    Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric OPEN  !!!", 14, 20);
    Graph_DrawText(&shooter.ui.fricClose, Operation_Change);
  }
  else // ����ر�Ħ����
  {
    shooter.fricOpenFlag = 0;
    shooter.fricMotor[0].targetSpeed = 0;
    shooter.fricMotor[1].targetSpeed = 0;
	  shooter.n20Motor.targetSpeed = 0;

    Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric Closed!!!", 14, 20);
    Graph_DrawText(&shooter.ui.fricClose, Operation_Change);
  }
  // �󲦸�����(������)��������
  static uint8_t triggerFlag = 0;
  if (rcInfo.left == 1 && triggerFlag == 0)
  {
    if (shooter.fricOpenFlag == 1) // Ħ���ֿ��� ������
    {
      shooter.workState = TRIGGER;
      triggerFlag = 1;
    }
  }
  else if (rcInfo.left != 1)
    triggerFlag = 0;
}

// ����Ԥ��  �ⲿ�ж�
// void User_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
////  if(GPIO_Pin==LOAD_Pin)
////  {
////    if(HAL_GPIO_ReadPin(GPIOI,LOAD_Pin)==0)  //�½���
////      shooter.bulletReadyFlag=true;
////    else
////      shooter.bulletReadyFlag=false;      //������
////  }
//}

// ��ѯ����Ԥ��״̬
void Shooter_InquireBulletState()
{
  if (HAL_GPIO_ReadPin(GPIOI, LOAD_Pin) == 0) // �͵�ƽ
    shooter.bulletReadyFlag = true;
  else
    shooter.bulletReadyFlag = false; // �ߵ�ƽ
}

/*************************RC�¼�**************************
���������ܼ���event����
*********************************************************/
// ����/ֹͣ��������
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  switch (event)
  {
  case KeyEvent_OnDown:            // ��������
    if (shooter.fricOpenFlag == 1) // Ħ���ֿ��� ������
    {
      shooter.workState = TRIGGER;
    }
    break;
  default:
    break;
  }
}
// �ֶ���/�ر�Ħ����
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  shooter.fricOpenFlag = !shooter.fricOpenFlag;
  if (shooter.fricOpenFlag == 1)
  {
    shooter.fricMotor[0].targetSpeed = -(shooter.fricSpd);
    shooter.fricMotor[1].targetSpeed = shooter.fricSpd;
	  shooter.n20Motor.targetSpeed = shooter.n20spd;

    Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric OPEN  !!!", 14, 20);
    Graph_DrawText(&shooter.ui.fricClose, Operation_Change);
  }
  else
  {
    shooter.fricMotor[0].targetSpeed = 0;
    shooter.fricMotor[1].targetSpeed = 0;
	  shooter.n20Motor.targetSpeed = 0;

    Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric Closed!!!", 14, 20);
    Graph_DrawText(&shooter.ui.fricClose, Operation_Change);
  }
}

// ���ĸ���������߶ȣ�10m��
void Shooter_ChangeLineIncHigh_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  if (key == Key_E)
    shooter.lineIncHigh += 2;
  else if (key == Key_D)
    shooter.lineIncHigh -= 2;

  char textBuf[30] = {0}; // �޸��Զ���UI
  sprintf(textBuf, "Line High = %03d", shooter.lineIncHigh);
  Graph_SetText(&shooter.ui.auxLineHigh, "ALH", Color_Orange, 2, 2, 100, 740, textBuf, 15, 20);
  Graph_DrawText(&shooter.ui.auxLineHigh, Operation_Change);

  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL2", Color_Pink, 2, 1, 944, 338 + shooter.lineIncHigh, 976, 338 + shooter.lineIncHigh); // 10m
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Change);
}

// ��x���
void Shooter_Distance_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  char textBuf[30] = {0}; // �޸��Զ���UI
  shooter.distance = g_nts_frame0.result.dis;
  sprintf(textBuf, "DISTANCE = %02.3f", shooter.distance);
  Graph_SetText(&shooter.ui.TofDistance, "TOF", Color_Green, 1, 2, 1060, 540, textBuf, 17, 20);
  Graph_DrawText(&shooter.ui.TofDistance, Operation_Change);
}

/************************freertos����**********************
����������freertos����ϵͳ����
**********************************************************/
// �������ص�
void Task_Shooter_Callback()
{

  if (rcInfo.wheel > 600) // ң���������л�ҡ�˿��ƻ���������
    shooter.rockerCtrl = true;
  else if (rcInfo.wheel < -600)
    shooter.rockerCtrl = false;

  if (shooter.rockerCtrl)
    Shooter_RockerCtrl();

  Shooter_InquireBulletState(); // ��ѯ����Ԥ��״̬
  Shooter_UpdateFricSpeed();
  // װ���߼�
  if (JUDGE_IsValid() == true && !JUDGE_GetShooterOutputState()) // �����Դû��������ϵ磩��û�в���ϵͳ
    shooter.workState = IDLE;
  if (!shooter.bulletReadyFlag && shooter.workState == IDLE) // ����δװ��
    shooter.workState = LOAD;
  else if (shooter.bulletReadyFlag && shooter.workState == LOAD)
    shooter.workState = IDLE;
  // ��ת����
  // ����Ƕ���Ŀ��Ƕ�����10������ж�ת�ж�
  if (ABS(shooter.triggerMotor.totalAngle - shooter.triggerMotor.targetAngle) > MOTOR_M3508_DGR2CODE(10) && shooter.workState != TRIGGER_REVERSE)
  {
    shooter.block.judgeCnt++;         // ��ת�ж�������++
    if (shooter.block.judgeCnt > 100) // �������ﵽһ��ֵ�����ж�Ϊ��ת��������ת
    {
      shooter.block.judgeCnt = 0;
      shooter.workState = TRIGGER_REVERSE;
    }
  }
  else // ��Ŀ��ֵ���С��10�ȣ�����״̬����������ת�ж�����������
  {
    shooter.block.judgeCnt = 0;
  }

  // ����
  switch (shooter.workState)
  {
  case IDLE:

    break;
  case LOAD:
    shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 0.005 / 7.0); // ÿ��ת��0.1/7Ȧ

    break;
  case TRIGGER:
    if (JUDGE_IsValid() == false || JUDGE_GetRemainHeat() >= 100) // δ��װ����ϵͳ �� ����ϵͳʣ����������100 ������
    {
      shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 1 / 7.0); // ����1/7Ȧ
      osDelay(100);
      shooter.workState = IDLE;
    }
    else
      shooter.workState = IDLE;
    break;
  case TRIGGER_REVERSE:
    // ������
    Beep_PlayNotes((Note[]){{T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}}, 3);
    Graph_DrawText(&shooter.ui.blockInfo, Operation_Add);                      // ��Ӷ�תUI����
    shooter.triggerMotor.targetAngle -= MOTOR_M3508_DGR2CODE(360 * 1.5 / 7.0); // ������򲦶�0.5/7Ȧ

    osDelay(500);
    shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 0.5 / 7.0); // ��ת0.5/7Ȧ
    shooter.workState = IDLE;
    Graph_DrawText(&shooter.ui.blockInfo, Operation_Delete); // ɾ����תui����
    break;
  default:
    break;
  }
}

#ifdef EN_SHOOTER_TASK
void os_ShooterCallback(void const *argument)
{
  PID_Clear(&shooter.triggerMotor.anglePID.inner);
  PID_Clear(&shooter.triggerMotor.anglePID.outer);

  osDelay(100);
  for (;;)
  {
    Task_Shooter_Callback();
    osDelay(10);
  }
}
#endif
