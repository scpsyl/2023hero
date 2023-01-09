#include "Shooter.h"
#include "RC.h"
#include <stdio.h>
#include "Userfreertos.h"
#include "judge.h"
//#include "usb_device.h"
#include "beep.h"
#include "user_tof.h"
#define EN_SHOOTER_TASK // 使能任务

Shooter shooter;

void Shooter_InitPID(void);
void Shooter_RegisterEvents(void);
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_ChangeLineIncHigh_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_Distance_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_InquireBulletState(void);
// 射击系统初始化
void Shooter_Init()
{

  shooter.rockerCtrl = false; // 初始状态键鼠控制

  shooter.fricSpd = 5450; // 默认摩擦轮速度5450rpm
	shooter.n20spd = 1000;//测试参数

  shooter.bulletReadyFlag = true; // 弹丸默认就绪（兼容无传感器的情况）
  shooter.fricOpenFlag = false;   // 摩擦轮默认开启

  Shooter_InitPID();        // 初始化电机pid
  Shooter_RegisterEvents(); // 注册事件
}
// 初始化shooter UI形状
/*在发送时须保证裁判系统串口已开启，并成功解析*/
void Shooter_UI_Init()
{
  /**************************射击辅助线 约 5.71坐标点/mm**************************************/
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL1", Color_Orange, 2, 1, 890, 425, 1030, 425); // 5m
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  // 10米左右击打前哨战线簇
  Graph_SetLine(&shooter.ui.auxiliaryLine, "095", Color_Orange, 2, 1, 944, 338 + 4, 976, 338 + 4); // 9.5m线
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL2", Color_Pink, 2, 1, 940, 338, 980, 338); // 10m线
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "105", Color_Orange, 2, 1, 944, 338 - 16, 976, 338 - 16); // 10.5m线
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL0", Color_Orange, 2, 1, 865, 540, 1045, 540); // 中心线
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);
  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL5", Color_Orange, 1, 1, 960, 200, 960, 583); // 竖线
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Add);

  Graph_SetText(&shooter.ui.auxLineHigh, "ALH", Color_Orange, 2, 2, 100, 740, "Line High = 000", 15, 20);
  Graph_DrawText(&shooter.ui.auxLineHigh, Operation_Add);
  Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric Closed!!!", 14, 20);
  Graph_DrawText(&shooter.ui.fricClose, Operation_Add);
  Graph_SetText(&shooter.ui.TofDistance, "TOF", Color_Green, 1, 2, 1060, 560, "DISTANCE = 00.000", 17, 20);
  Graph_DrawText(&shooter.ui.TofDistance, Operation_Add);
  Graph_SetText(&shooter.ui.blockInfo, "TBK", Color_Yellow, 2, 2, 100, 710, "Trig Blocked", 12, 20);
}
/************内部工具函数*****************/
// 初始化PID参数
void Shooter_InitPID()
{
  Motor_StartCalcAngle(&shooter.triggerMotor); // 初始化电机角度累计
  PID_Init(&shooter.triggerMotor.anglePID.inner, 15, 0, 0, 7800, 20000);
  PID_Init(&shooter.triggerMotor.anglePID.outer, 0.3, 0, 0, 0, 7000);
  PID_Init(&shooter.fricMotor[0].speedPID, 60, 0, 0, 10000, 20000);
  PID_Init(&shooter.fricMotor[1].speedPID, 60, 0, 0, 10000, 20000);
	PID_Init(&shooter.n20Motor.speedPID,200,0.055,0,10000,10000);
}
// 注册事件
void Shooter_RegisterEvents()
{
  // 左键按下抬起开关拨弹
  RC_Register(Key_Left, CombineKey_None, KeyEvent_OnDown, Shooter_SwitchState_KeyCallback);
  RC_Register(Key_Left, CombineKey_None, KeyEvent_OnUp, Shooter_SwitchState_KeyCallback);
  RC_Register(Key_Left, CombineKey_None, KeyEvent_OnPressing, Shooter_SwitchState_KeyCallback);
  // F开启摩擦轮
  RC_Register(Key_F, CombineKey_None, KeyEvent_OnDown, Shooter_StartFric_KeyCallback);
  // shift+E/D增减10m辅助线高度
  RC_Register(Key_E | Key_D, CombineKey_Shift, KeyEvent_OnDown, Shooter_ChangeLineIncHigh_KeyCallback);
  // 按x显示距离
  RC_Register(Key_X, CombineKey_None, KeyEvent_OnDown, Shooter_Distance_KeyCallback);
}

// 根据裁判系统信息更新摩擦轮转速
void Shooter_UpdateFricSpeed()
{
  if (JUDGE_GetShootSpeedLimit() >= 16)
    shooter.fricSpd = 5400; // 15.5m/s
  else
    shooter.fricSpd = 3800; // 9.2m/s
}

// 遥感控制
void Shooter_RockerCtrl()
{
  if (rcInfo.right == 1) // 右拨杆向上则打开摩擦轮
  {
    shooter.fricOpenFlag = 1;
    shooter.fricMotor[0].targetSpeed = -(shooter.fricSpd);
    shooter.fricMotor[1].targetSpeed = shooter.fricSpd;
	  shooter.n20Motor.targetSpeed = shooter.n20spd;

    Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric OPEN  !!!", 14, 20);
    Graph_DrawText(&shooter.ui.fricClose, Operation_Change);
  }
  else // 否则关闭摩擦轮
  {
    shooter.fricOpenFlag = 0;
    shooter.fricMotor[0].targetSpeed = 0;
    shooter.fricMotor[1].targetSpeed = 0;
	  shooter.n20Motor.targetSpeed = 0;

    Graph_SetText(&shooter.ui.fricClose, "FCS", Color_Orange, 2, 2, 100, 680, "Fric Closed!!!", 14, 20);
    Graph_DrawText(&shooter.ui.fricClose, Operation_Change);
  }
  // 左拨杆向上(上升沿)则开启拨弹
  static uint8_t triggerFlag = 0;
  if (rcInfo.left == 1 && triggerFlag == 0)
  {
    if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
    {
      shooter.workState = TRIGGER;
      triggerFlag = 1;
    }
  }
  else if (rcInfo.left != 1)
    triggerFlag = 0;
}

// 弹丸预置  外部中断
// void User_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
////  if(GPIO_Pin==LOAD_Pin)
////  {
////    if(HAL_GPIO_ReadPin(GPIOI,LOAD_Pin)==0)  //下降沿
////      shooter.bulletReadyFlag=true;
////    else
////      shooter.bulletReadyFlag=false;      //上升沿
////  }
//}

// 查询弹丸预置状态
void Shooter_InquireBulletState()
{
  if (HAL_GPIO_ReadPin(GPIOI, LOAD_Pin) == 0) // 低点平
    shooter.bulletReadyFlag = true;
  else
    shooter.bulletReadyFlag = false; // 高电平
}

/*************************RC事件**************************
以下任务受键鼠event调度
*********************************************************/
// 触发/停止拨弹工作
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  switch (event)
  {
  case KeyEvent_OnDown:            // 单发拨弹
    if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
    {
      shooter.workState = TRIGGER;
    }
    break;
  default:
    break;
  }
}
// 手动打开/关闭摩擦轮
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

// 更改辅助线增益高度（10m）
void Shooter_ChangeLineIncHigh_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  if (key == Key_E)
    shooter.lineIncHigh += 2;
  else if (key == Key_D)
    shooter.lineIncHigh -= 2;

  char textBuf[30] = {0}; // 修改自定义UI
  sprintf(textBuf, "Line High = %03d", shooter.lineIncHigh);
  Graph_SetText(&shooter.ui.auxLineHigh, "ALH", Color_Orange, 2, 2, 100, 740, textBuf, 15, 20);
  Graph_DrawText(&shooter.ui.auxLineHigh, Operation_Change);

  Graph_SetLine(&shooter.ui.auxiliaryLine, "AL2", Color_Pink, 2, 1, 944, 338 + shooter.lineIncHigh, 976, 338 + shooter.lineIncHigh); // 10m
  Graph_DrawLine(&shooter.ui.auxiliaryLine, Operation_Change);
}

// 按x测距
void Shooter_Distance_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
  char textBuf[30] = {0}; // 修改自定义UI
  shooter.distance = g_nts_frame0.result.dis;
  sprintf(textBuf, "DISTANCE = %02.3f", shooter.distance);
  Graph_SetText(&shooter.ui.TofDistance, "TOF", Color_Green, 1, 2, 1060, 540, textBuf, 17, 20);
  Graph_DrawText(&shooter.ui.TofDistance, Operation_Change);
}

/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/
// 射击任务回调
void Task_Shooter_Callback()
{

  if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制还是鼠标控制
    shooter.rockerCtrl = true;
  else if (rcInfo.wheel < -600)
    shooter.rockerCtrl = false;

  if (shooter.rockerCtrl)
    Shooter_RockerCtrl();

  Shooter_InquireBulletState(); // 查询弹丸预置状态
  Shooter_UpdateFricSpeed();
  // 装弹逻辑
  if (JUDGE_IsValid() == true && !JUDGE_GetShooterOutputState()) // 发射电源没有输出（断电）或没有裁判系统
    shooter.workState = IDLE;
  if (!shooter.bulletReadyFlag && shooter.workState == IDLE) // 弹丸未装载
    shooter.workState = LOAD;
  else if (shooter.bulletReadyFlag && shooter.workState == LOAD)
    shooter.workState = IDLE;
  // 堵转处理
  // 电机角度与目标角度相差超过10度则进行堵转判定
  if (ABS(shooter.triggerMotor.totalAngle - shooter.triggerMotor.targetAngle) > MOTOR_M3508_DGR2CODE(10) && shooter.workState != TRIGGER_REVERSE)
  {
    shooter.block.judgeCnt++;         // 堵转判定计数器++
    if (shooter.block.judgeCnt > 100) // 计数器达到一定值，则判定为堵转，触发反转
    {
      shooter.block.judgeCnt = 0;
      shooter.workState = TRIGGER_REVERSE;
    }
  }
  else // 与目标值相差小于10度，拨弹状态正常，将堵转判定计数器归零
  {
    shooter.block.judgeCnt = 0;
  }

  // 拨弹
  switch (shooter.workState)
  {
  case IDLE:

    break;
  case LOAD:
    shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 0.005 / 7.0); // 每次转动0.1/7圈

    break;
  case TRIGGER:
    if (JUDGE_IsValid() == false || JUDGE_GetRemainHeat() >= 100) // 未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
    {
      shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 1 / 7.0); // 拨动1/7圈
      osDelay(100);
      shooter.workState = IDLE;
    }
    else
      shooter.workState = IDLE;
    break;
  case TRIGGER_REVERSE:
    // 嘀嘀嘀
    Beep_PlayNotes((Note[]){{T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}}, 3);
    Graph_DrawText(&shooter.ui.blockInfo, Operation_Add);                      // 添加堵转UI提醒
    shooter.triggerMotor.targetAngle -= MOTOR_M3508_DGR2CODE(360 * 1.5 / 7.0); // 电机反向拨动0.5/7圈

    osDelay(500);
    shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 0.5 / 7.0); // 正转0.5/7圈
    shooter.workState = IDLE;
    Graph_DrawText(&shooter.ui.blockInfo, Operation_Delete); // 删除堵转ui提醒
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
