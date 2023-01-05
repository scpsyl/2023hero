#ifndef _BEEP_H_
#define _BEEP_H_

#include "main.h"

// �����˶ȶ�Ӧ������(us)
enum
{
  // ����
  T_None = 0,
  // �Ͱ˶�
  T_L1 = 3822,
  T_L2 = 3405,
  T_L3 = 3034,
  T_L4 = 2863,
  T_L5 = 2551,
  T_L6 = 2272,
  T_L7 = 2052,
  // �а˶�
  T_M1 = 1911,
  T_M2 = 1703,
  T_M3 = 1517,
  T_M4 = 1432,
  T_M5 = 1276,
  T_M6 = 1136,
  T_M7 = 1012,
  // �߰˶�
  T_H1 = 956,
  T_H2 = 851,
  T_H3 = 758,
  T_H4 = 716,
  T_H5 = 638,
  T_H6 = 568,
  T_H7 = 506
};

// ����ʱֵ��Ӧ��ʱ��(ms) (4/4�� �ٶ�=100)
enum
{
  D_Whole = 2400,   // ȫ����
  D_Half = 1200,    // ��������
  D_Quarter = 600,  // �ķ�����
  D_Eighth = 300,   // �˷�����
  D_Sixteenth = 150 // ʮ��������
};

// һ������
typedef struct
{
  uint16_t period;   // ��������
  uint16_t duration; // ����ʱ��
} Note;

// ��������ʼ��
void Beep_Init(void);
// ����һ������
void Beep_PlayOneNote(Note *note);
// ����һ������
void Beep_PlayNotes(Note notes[], uint8_t noteNum);
// ����ʽ������������
void Beep_Block_PlayAll(void);
// ����������ص�
void Task_Beep_Callback(void);

#endif
