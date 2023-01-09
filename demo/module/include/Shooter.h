#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include "Moto.h"
#include "stdbool.h"
#include "Graphics.h"
enum
{
  IDLE = 0,
  LOAD,
  TRIGGER,
  TRIGGER_REVERSE
};

typedef struct
{
  uint8_t workState;        // ����״̬
  bool rockerCtrl;          // ����Ƿ���ҡ�˿���
  SingleMotor triggerMotor; // ���������Ħ���ֵ��
  SingleMotor fricMotor[2];
  SingleMotor n20Motor; // ��λ���
  bool bulletReadyFlag; // �ӵ���λ��־
  bool fricOpenFlag;    // Ħ���ֿ�����־
  int16_t fricSpd;      // Ħ�����ٶ�
  int16_t n20spd;       // ��λ���ת��
  int16_t lineIncHigh;  // 10m�����߸߶�����
  float distance;       // ��� ��������
  struct
  {
    uint16_t judgeCnt, reverseCnt; // ��ת�ж�������,��ת������
    uint16_t fric_judgeCnt, fric_reverseCnt;
  } block; // ��ת�����������
  struct
  {
    Line auxiliaryLine; // ׼�ĸ����� ���� ˮƽ0 5m 10m 15m 20m
    Text auxLineHigh;   // 10m�߸߶Ƚ���
    Text TofDistance;   // tof����
    Text blockInfo;     // ��ʾ������ת
    Text fricClose;     // ��ʾĦ���ֹر�
  } ui;
} Shooter;

extern Shooter shooter;

void Shooter_Init(void);
void Shooter_UI_Init(void);
void User_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif
