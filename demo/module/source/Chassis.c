/**
 * @file Chassis.c
 * @author Screeps
 * @brief  底盘文件
 * @version 0.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Chassis.h"
#include "arm_math.h"
#include "rc.h"
#include "UserFreertos.h"
#include "gimbal.h"
#include "super_cap.h"
#include "detect.h"
#include "shooter.h"

#define EN_CHASSIS_TASK // 使能任务

// 内部工具接口声明
void Chassis_PIDInit(void);
void Chassis_RegisterEvents(void);
char *Chassis_GetModeText(void);
void Chassis_UpdateSpeedLimit(void);
void Chassis_UpdateSlope(void);
void Chassis_RockerCtrl(void);
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Task_Chassis_Callback(void);

Chassis chassis = {0};

/**
 * @brief 底盘初始化
 *
 */
void Chassis_Init()
{
	// 底盘尺寸信息（用于解算轮速）
	chassis.info.wheelbase = 466; // 轴距暂时测试正常
	chassis.info.wheeltrack = 461;
	chassis.info.wheelRadius = 76;
	chassis.info.offsetX = 0;
	chassis.info.offsetY = 0;
	// 移动参数初始化
	chassis.move.maxVx = 2000;
	chassis.move.maxVy = 2000;
	chassis.move.maxVw = 2;
	chassis.speedRto = 1;
	// 旋转参数初始化
	chassis.rotate.InitAngle = 3000; // 平行时yaw轴电机角度  yaw不稳定 还需要调
	// 斜坡函数初始化
	Slope_Init(&chassis.move.xSlope, 5, 5);
	Slope_Init(&chassis.move.ySlope, 5, 5);
	// 电机pid初始化
	Chassis_PIDInit();
	// 注册事件
	Chassis_RegisterEvents();
}
/**
 * @brief 初始化chassis UI形状
 *
 * @attention 在发送时须保证裁判系统串口已开启，并成功解析
 */
void Chassis_UI_Init()
{
	Graph_SetText(&chassis.ui.mode, "CHA", Color_Orange, 2, 0, 100, 800, "Chassis Mode = Follow", 21, 20);
	Graph_DrawText(&chassis.ui.mode, Operation_Add);
	Graph_SetText(&chassis.ui.speed, "SPD", Color_Orange, 2, 0, 100, 830, "Chassis Speed = Fast", 20, 20);
	Graph_DrawText(&chassis.ui.speed, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT1", Color_Yellow, 2, 0, 1100, 205, 1360, 205); // 车宽线（镜头水平）
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT2", Color_Yellow, 2, 0, 560, 205, 820, 205);
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT3", Color_Yellow, 2, 0, 1360, 205, 1420, 175);
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);
	Graph_SetLine(&chassis.ui.outline, "OT4", Color_Yellow, 2, 0, 480, 155, 560, 205);
	Graph_DrawLine(&chassis.ui.outline, Operation_Add);

	Graph_SetText(&chassis.ui.super_cap, "CAP", Color_Orange, 2, 0, 100, 650, "CAP ENERGY= 100 %", 17, 20);
	Graph_DrawText(&chassis.ui.super_cap, Operation_Add);
}

/************内部工具函数*****************/
/**
 * @brief 底盘pid参数初始化
 *
 */
void Chassis_PIDInit()
{
	// 速度环
	PID_Init(&chassis.motors[0].speedPID, 20, 0, 0, 8000, 10000);
	PID_Init(&chassis.motors[1].speedPID, 20, 0, 0, 8000, 10000);
	PID_Init(&chassis.motors[2].speedPID, 20, 0, 0, 8000, 10000);
	PID_Init(&chassis.motors[3].speedPID, 20, 0, 0, 8000, 10000);

	PID_Init(&chassis.rotate.pid, 0.04, 0, 0, 0.5, 0.5);
	PID_SetDeadzone(&chassis.rotate.pid, 2);
}

/**
 * @brief 注册所有事件
 *
 */
void Chassis_RegisterEvents()
{
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnDown, Chassis_Move_KeyCallback); // WASD按下，底盘移动
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnUp, Chassis_Stop_KeyCallback);	// WASD松开，底盘停止移动
	RC_Register(Key_Q | Key_E | Key_R, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchMode_KeyCallback);	// QER切换底盘模式
	RC_Register(Key_C, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchSpeed_KeyCallback);					// 按下c下蹲
}
/**
 * @brief 根据最大功率设定移动速度
 *
 */
void Chassis_UpdateSpeedLimit()
{
	uint8_t maxPower = JUDGE_GetChassisPowerLimit(); // 获取底盘功率限制
	float ratio = (maxPower - 50) / (120 - 50);
	if (JUDGE_IsValid() != true) // 数据无效或未安装裁判系统
		ratio = 1;
	chassis.move.maxVx = (1900 + 1600 * ratio);
	chassis.move.maxVy = (1900 + 1600 * ratio);
	chassis.move.maxVw = (2.375f + 0.8f * ratio);
	Slope_SetStep(&chassis.move.xSlope, 4.75f + 4 * ratio);
	Slope_SetStep(&chassis.move.ySlope, 4.75f + 4 * ratio);
	PID_SetMaxOutput(&chassis.rotate.pid, chassis.move.maxVw); // 设定最大旋转速度
}

/**
 * @brief 更新斜坡计算速度
 *
 */
void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
}
/**
 * @brief 摇杆控制底盘
 *
 */
void Chassis_RockerCtrl()
{
	// 左拨杆拨到下方进入小陀螺，拨回来进入跟随模式
	if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left != 2)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown); // 模拟操作手按下按键
	else if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown);
	// 设定移动方向及速度
	Slope_SetTarget(&chassis.move.xSlope, (float)rcInfo.ch3 * chassis.move.maxVx / 660); // 设定斜坡目标值
	Slope_SetTarget(&chassis.move.ySlope, (float)rcInfo.ch4 * chassis.move.maxVy / 660);
}
/**
 * @brief 获取底盘模式UI文字
 *
 * @return char* 底盘模式提示信息
 */
char *Chassis_GetModeText()
{
	switch (chassis.rotate.mode)
	{
	case ChassisMode_Spin:
		return "Chassis Mode = Spin  ";
	case ChassisMode_Follow:
		return "Chassis Mode = Follow";
	case ChassisMode_Snipe_10:
		return "Chassis Mode = Snipe1";
	case ChassisMode_Snipe_20:
		return "Chassis Mode = Snipe2";
	default:
		return NULL;
	}
}
/************外部接口函数*******************/

/**
 * @brief 计算电机输出限制比例(缓冲能量不足时<1)
 *
 * @return float 电机限制百分比
 * @attention 底盘缓冲能量值>30情况下不限制电机输出
 */
float Chassis_GetOutputRatio()
{
	uint16_t powerBuf = JUDGE_GetPowerBuffer(); // 获取缓冲能量值

	if (JUDGE_IsValid() != true) // 裁判系统数据无效
		powerBuf = 40;

	if (powerBuf > 40) // 若大于30则不限制电机输出
	{
		return 1;
	}
	else
	{
		return (powerBuf / 40.0f) * (powerBuf / 40.0f); // 缓冲能量小于30则按比例限制电机输出
	}
}
/*************************RC事件**************************
以下任务受键鼠event调度
*********************************************************/
// WASD移动
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
		Slope_SetTarget(&chassis.move.ySlope, -chassis.move.maxVy * chassis.speedRto); // 设定斜坡目标值
		break;
	case Key_S:
		Slope_SetTarget(&chassis.move.ySlope, chassis.move.maxVy * chassis.speedRto);
		break;
	case Key_D:
		Slope_SetTarget(&chassis.move.xSlope, -chassis.move.maxVx * chassis.speedRto);
		break;
	case Key_A:
		Slope_SetTarget(&chassis.move.xSlope, chassis.move.maxVx * chassis.speedRto);
		break;
	default:
		break;
	}
}

// 停止移动按键回调
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
	case Key_S:
		Slope_SetTarget(&chassis.move.ySlope, 0); // 设定斜坡目标值为0
		break;
	case Key_A:
	case Key_D:
		Slope_SetTarget(&chassis.move.xSlope, 0);
		break;
	default:
		break;
	}
}
// 慢/快速移动按键回调
void Chassis_SwitchSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (chassis.speedRto == 1)
		chassis.speedRto = 0.5;
	else
		chassis.speedRto = 1;
	Graph_SetText(&chassis.ui.speed, "SPD", Color_Orange, 2, 0, 100, 830, chassis.speedRto == 1 ? "Chassis Speed = Fast" : "Chassis Speed = Slow", 20, 20);
	Graph_DrawText(&chassis.ui.speed, Operation_Change);
}

// 切换底盘模式
void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	float H;
	float V;
	float T;

	if (chassis.rotate.mode != ChassisMode_Follow) //  Q/E/R切换模式 再次按下回到跟随模式
	{
		chassis.rotate.mode = ChassisMode_Follow;
		chassis.speedRto = 1;
	}
	else
	{
		switch (key)
		{
		case Key_Q: // 小陀螺模式
			PID_Clear(&chassis.rotate.pid);
			chassis.rotate.mode = ChassisMode_Spin;
			break;
		case Key_E: // 狙击模式10m
			chassis.rotate.mode = ChassisMode_Snipe_10;
			if (shooter.distance > 1.5f)
			{
				H = 0.2f;
				V = 15.5f;
				T = 0.020825f * pow(V, 2) - 0.204082f * H - 0.004165f * sqrt(25 * pow(V, 4) - 490.0f * H * pow(V, 2) - 2401.0f * pow(shooter.distance, 2));
				T = sqrt(T);
				gimbal.pitch.targetAngle = acos(shooter.distance / (V * T)) * 180.0f / PI;
			}
			else
			{
				gimbal.pitch.targetAngle = 14.7f;
			}
			LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
			chassis.speedRto = 0.1; // 移速
			break;
		case Key_R: // 狙击模式20m
			chassis.rotate.mode = ChassisMode_Snipe_20;
			gimbal.pitch.targetAngle = (-36); // shooter.distance  //
			chassis.speedRto = 0.1;			  // 移速
		default:
			break;
		}
	}
	Graph_SetText(&chassis.ui.mode, "CHA", Color_Orange, 2, 0, 100, 800, Chassis_GetModeText(), 21, 20);
	Graph_DrawText(&chassis.ui.mode, Operation_Change);
}

/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/

// 底盘任务回调函数
void Task_Chassis_Callback()
{
	Chassis_UpdateSpeedLimit();
	chassis.rotate.nowAngle = gimbal.yawMotor.angle;
	chassis.rotate.relativeAngle = MOTOR_M6020_CODE2DGR(chassis.rotate.nowAngle - chassis.rotate.InitAngle);

	if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制/键盘控制
		chassis.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		chassis.rockerCtrl = false;

	if (chassis.rockerCtrl) // 摇杆控制
		Chassis_RockerCtrl();

	/***计算底盘平移速度****/

	Chassis_UpdateSlope(); // 更新运动斜坡函数数据

	// 将云台坐标系下平移速度解算到底盘平移速度(根据云台偏离角)
	float gimbalAngleSin = arm_sin_f32(chassis.rotate.relativeAngle * PI / 180);
	float gimbalAngleCos = arm_cos_f32(chassis.rotate.relativeAngle * PI / 180);
	chassis.move.vx = Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin;
	chassis.move.vy = -Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos;

	/***计算底盘旋转速度***/

	if (chassis.rotate.mode == ChassisMode_Follow) // 跟随模式
	{
		PID_SingleCalc(&chassis.rotate.pid, 0, chassis.rotate.relativeAngle);
		chassis.move.vw = chassis.rotate.pid.output;
	}
	else if (chassis.rotate.mode == ChassisMode_Spin) // 小陀螺模式
	{
		float ratio = 1 - (ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy) / 2;
		chassis.move.vw = chassis.move.maxVw * ratio * chassis.speedRto;
	}
	else if (chassis.rotate.mode == ChassisMode_Snipe_10 || chassis.rotate.mode == ChassisMode_Snipe_20) // 狙击模式
	{
		PID_SingleCalc(&chassis.rotate.pid, 45, chassis.rotate.relativeAngle); // 底盘横置  某方向45°可能和上赛季反向还没测试
		chassis.move.vw = chassis.rotate.pid.output;
	}
	/***解算各轮子转速****/
	float rotateRatio[4];
	rotateRatio[0] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f - chassis.info.offsetY + chassis.info.offsetX;
	rotateRatio[1] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f - chassis.info.offsetY - chassis.info.offsetX;
	rotateRatio[2] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f + chassis.info.offsetY + chassis.info.offsetX;
	rotateRatio[3] = (chassis.info.wheelbase + chassis.info.wheeltrack) / 2.0f + chassis.info.offsetY - chassis.info.offsetX;
	int16_t wheelRPM[4];
	wheelRPM[0] = (chassis.move.vx + chassis.move.vy - chassis.move.vw * rotateRatio[0]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19;	// FL
	wheelRPM[1] = -(-chassis.move.vx + chassis.move.vy + chassis.move.vw * rotateRatio[1]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19; // FR
	wheelRPM[2] = (-chassis.move.vx + chassis.move.vy - chassis.move.vw * rotateRatio[2]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19;	// BL
	wheelRPM[3] = -(chassis.move.vx + chassis.move.vy + chassis.move.vw * rotateRatio[3]) * 60 / (2 * PI * chassis.info.wheelRadius) * 19;	// BR
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis.motors[i].targetSpeed = wheelRPM[i];
	}
}

#ifdef EN_CHASSIS_TASK
void os_ChassisCallback(void const *argument)
{

	for (;;)
	{
		Task_Chassis_Callback();
		osDelay(2);
	}
}
#endif
