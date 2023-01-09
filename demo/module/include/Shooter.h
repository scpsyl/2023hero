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
  uint8_t workState;        // 工作状态
  bool rockerCtrl;          // 标记是否用摇杆控制
  SingleMotor triggerMotor; // 拨弹电机和摩擦轮电机
  SingleMotor fricMotor[2];
  SingleMotor n20Motor; // 限位电机
  bool bulletReadyFlag; // 子弹到位标志
  bool fricOpenFlag;    // 摩擦轮开启标志
  int16_t fricSpd;      // 摩擦轮速度
  int16_t n20spd;       // 限位电机转速
  int16_t lineIncHigh;  // 10m辅助线高度修正
  float distance;       // 测距 用来补偿
  struct
  {
    uint16_t judgeCnt, reverseCnt; // 堵转判定计数器,反转计数器
    uint16_t fric_judgeCnt, fric_reverseCnt;
  } block; // 堵转处理相关数据
  struct
  {
    Line auxiliaryLine; // 准心辅助线 竖线 水平0 5m 10m 15m 20m
    Text auxLineHigh;   // 10m线高度较正
    Text TofDistance;   // tof距离
    Text blockInfo;     // 提示拨弹堵转
    Text fricClose;     // 提示摩擦轮关闭
  } ui;
} Shooter;

extern Shooter shooter;

void Shooter_Init(void);
void Shooter_UI_Init(void);
void User_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif
