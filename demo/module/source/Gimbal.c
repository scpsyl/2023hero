#include "Gimbal.h"
#include "RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Detect.h"
#include <stdio.h>
#include "UserFreertos.h"
#include "beep.h"
#define EN_GIMBAL_TASK // 使能任务

void Gimbal_InitPID(void);
void Gimbal_RegisterEvents(void);
void Gimbal_UpdataAngle(void);
void Gimbal_MouseCtrl(void);
void Gimbal_VisionCtrl(void);
void Gimbal_RockerCtrl(void);
void Gimbal_SwitchVision_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_SelectVisionTarget_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_ChangeVisionPitchInc_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_ManualAdjustYaw_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_CalibrationUI_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Gimbal_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

Gimbal gimbal = {0};
float temp;
/********************初始化************************/
// 初始化云台
void Gimbal_Init()
{

	gimbal.pitch.pitchMax = 42; // 设定pitch角度限幅
	gimbal.pitch.pitchMin = -19;

	gimbal.pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
	gimbal.pitch.targetAngle = gimbal.pitch.initAngle;

	gimbal.yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
	gimbal.yaw.targetAngle = gimbal.yaw.initAngle;

	gimbal.visionEnable = false; // 自瞄默认关闭

	gimbal.rockerCtrl = false; // 默认使用键盘控制

	gimbal.visionPitchIncLevel = 0; // pitch增益初值
	gimbal.manualYawOffset = 0;		// yaw偏移值

	Filter_InitAverFilter(&gimbal.Mouse.yawFilter, 10); // 键盘数据均值滤波 平滑鼠标数据
	Filter_InitAverFilter(&gimbal.Mouse.pitchFilter, 10);

	Filter_InitAverFilter(&gimbal.visionFilter.find, 50); // 视觉数据均值滤波

	Gimbal_InitPID();		 // 初始化PID参数
	Gimbal_RegisterEvents(); // 注册事件
}

// 初始化gimbal UI形状
void Gimbal_UI_Init()
{
	Graph_SetCircle(&gimbal.ui.visionState, "VSN", Color_White, 5, 2, 960, 540, 30);
	Graph_SetText(&gimbal.ui.pitchIncLevel, "PTH", Color_Orange, 2, 3, 100, 770, "Pitch Level = 000", 17, 20);
	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Add);
	Graph_DrawText(&gimbal.ui.pitchIncLevel, Operation_Add);
}

/************内部工具函数*****************/

// 初始化PID参数
void Gimbal_InitPID()
{
	/*pitch由陀螺仪控制*/
	PID_Init(&gimbal.pitch.imuPID.inner, 25, 0, 200, 0, 20000);
	PID_Init(&gimbal.pitch.imuPID.outer, -500, 0, 0, 500, 20000);

	/*yaw由陀螺仪控制*/
	PID_Init(&gimbal.yaw.imuPID.inner, 30, 0, 200, 0, 20000);
	PID_Init(&gimbal.yaw.imuPID.outer, -600, 0, 0, 500, 20000);

	PID_Init(&gimbal.pitchVisionPID, 0.012, 0, 0, 0.5, 0.5);
	PID_Init(&gimbal.yawVisionPID, 0.021, 0, 0.1, 0.5, 0.5);
	//  PID_Init(&gimbal.yawVisionPID,0,0,0,0.5,0.5);
}

// 注册事件
void Gimbal_RegisterEvents()
{
	RC_Register(Key_Right, CombineKey_None, KeyEvent_OnDown, Gimbal_SwitchVision_KeyCallback); // 鼠标右键开关自瞄
	RC_Register(Key_Right, CombineKey_None, KeyEvent_OnUp, Gimbal_SwitchVision_KeyCallback);
	RC_Register(Key_Z | Key_X | Key_C, CombineKey_Shift, KeyEvent_OnDown, Gimbal_SelectVisionTarget_KeyCallback); // shift+z\x\c选择自瞄目标
	RC_Register(Key_Q | Key_A, CombineKey_Shift, KeyEvent_OnDown, Gimbal_ChangeVisionPitchInc_KeyCallback);		  // shift+q\a调节pitch增益
	RC_Register(Key_Z | Key_X, CombineKey_Shift, KeyEvent_OnPressing, Gimbal_ManualAdjustYaw_KeyCallback);		  // 按住shift+z\x调节yaw偏移
	RC_Register(Key_R, CombineKey_Ctrl, KeyEvent_OnDown, Gimbal_Return_KeyCallback);							  // ctrl R 一键调头
}

// 更新陀螺仪角度并累积yaw角度
void Gimbal_UpdataAngle()
{
	gimbal.yaw.gyro = User_Imu_Get_Z_Gyro();
	gimbal.yaw.angle = User_Imu_GetYaw();
	/*c板安装方向与官方imu三轴定义方向不同 yaw一致 pitch roll相反*/
	gimbal.pitch.gyro = User_Imu_Get_X_Gyro();
	gimbal.pitch.angle = User_Imu_GetRoll();

	float dAngle = 0;
	if (gimbal.yaw.angle - gimbal.yaw.lastAngle < -270)
		dAngle = gimbal.yaw.angle + (360 - gimbal.yaw.lastAngle);
	else if (gimbal.yaw.angle - gimbal.yaw.lastAngle > 270)
		dAngle = -gimbal.yaw.lastAngle - (360 - gimbal.yaw.angle);
	else
		dAngle = gimbal.yaw.angle - gimbal.yaw.lastAngle;
	gimbal.yaw.totalAngle += dAngle;
	gimbal.yaw.lastAngle = gimbal.yaw.angle;
}

// 鼠标控制云台
void Gimbal_MouseCtrl()
{
	if (chassis.rotate.mode == ChassisMode_Snipe_10 || chassis.rotate.mode == ChassisMode_Snipe_20)
	{
		gimbal.Mouse.yawDPI = 0.001 * 0.1;
		gimbal.Mouse.pitchDPI = 0.0005 * 0.1;
	}
	else
	{
		gimbal.Mouse.yawDPI = 0.001;
		gimbal.Mouse.pitchDPI = 0.0005;
	}
	gimbal.yaw.targetAngle -= Filter_AverCalc(&gimbal.Mouse.yawFilter, rcInfo.mouse.x * gimbal.Mouse.yawDPI);		// yaw修改
	gimbal.pitch.targetAngle -= Filter_AverCalc(&gimbal.Mouse.pitchFilter, rcInfo.mouse.y * gimbal.Mouse.pitchDPI); // pitch修改
	LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);									// pitch限幅
}

// 视觉控制云台
void Gimbal_VisionCtrl()
{
	PID_SingleCalc(&gimbal.yawVisionPID, gimbal.manualYawOffset, visionRecv.yaw);
	gimbal.yaw.targetAngle -= gimbal.yawVisionPID.output;
	PID_SingleCalc(&gimbal.pitchVisionPID, gimbal.visionPitchIncLevel, visionRecv.pitch);
	gimbal.pitch.targetAngle -= gimbal.pitchVisionPID.output; // 旋转云台pitch
	LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}

// 摇杆控制云台
void Gimbal_RockerCtrl()
{
	gimbal.yaw.targetAngle -= rcInfo.ch1 * 0.35 / 660.0f;	// yaw
	gimbal.pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
	LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}

/************外部接口函数*******************/
// 视觉PC掉线回调
void Gimbal_VisionLostCallback()
{
	gimbal.ui.visionState.color = Color_White;
	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
	visionRecv.pitch = visionRecv.yaw = visionRecv.distance = 0;
	visionRecv.find = 0;
}

// 视觉恢复连接回调
void Gimbal_VisionRecoverCallback()
{
	gimbal.ui.visionState.color = Color_Yellow;
	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
}

/*************************RC事件**************************
以下任务受键鼠event调度
*********************************************************/

// 开关自瞄键盘回调
void Gimbal_SwitchVision_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (Detect_IsDeviceLost(DeviceID_PC))
		return;
	switch (event)
	{
	case KeyEvent_OnDown:
		gimbal.visionEnable = true;
		break;
	case KeyEvent_OnUp:
		gimbal.visionEnable = false;
		break;
	default:
		break;
	}
}

// 加减视觉PITCH增益
void Gimbal_ChangeVisionPitchInc_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (key == Key_Q)
		gimbal.visionPitchIncLevel++;
	else if (key == Key_A)
		gimbal.visionPitchIncLevel--;

	char textBuf[30] = {0};

	sprintf(textBuf, "Pitch Level = %03d", gimbal.visionPitchIncLevel);
	Graph_SetText(&gimbal.ui.pitchIncLevel, "PTH", Color_Orange, 2, 0, 100, 770, textBuf, 17, 20);
	Graph_DrawText(&gimbal.ui.pitchIncLevel, Operation_Change);
}

// 选择自瞄目标
void Gimbal_SelectVisionTarget_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_Z:
		Vision_SetMode(VisionMode_AimAt1);
		break;
	case Key_X:
		Vision_SetMode(VisionMode_AimAt3);
		break;
	case Key_C:
		Vision_SetMode(VisionMode_AimAt6);
		break;
	default:
		break;
	}
}

// 手动调整yaw角度偏移
void Gimbal_ManualAdjustYaw_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (key == Key_Z)
		gimbal.manualYawOffset -= 1.0f;
	else if (key == Key_X)
		gimbal.manualYawOffset += 1.0f;
}

// 一键调头
void Gimbal_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	Slope_SetTarget(&chassis.move.ySlope, 0);
	Slope_SetTarget(&chassis.move.xSlope, 0); // 刹车
	gimbal.yaw.targetAngle += 180;
}

/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/
// 云台控制任务
void Task_Gimbal_Callback()
{
	if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制还是鼠标控制
		gimbal.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		gimbal.rockerCtrl = false;

	if (rcInfo.wheel > 600) // 遥控器模式下拨轮拨到底端开启自瞄
		gimbal.visionEnable = true;
	else if (rcInfo.wheel < 600 && gimbal.rockerCtrl) // 遥控器模式下拨轮不在底端关闭自瞄
		gimbal.visionEnable = false;

	// 对识别状态进行滤波，防止偶尔的误识别
	float visionFindAver = Filter_AverCalc(&gimbal.visionFilter.find, visionRecv.find);

	// 根据识别状态绘制UI
	if (gimbal.ui.visionState.color != Color_Green && visionFindAver > 0.5f)
	{
		gimbal.ui.visionState.color = Color_Green; // 绿色中心圈表示已识别到
		Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
	}
	else if (gimbal.ui.visionState.color != Color_Yellow && visionFindAver < 0.5f)
	{
		gimbal.ui.visionState.color = Color_Yellow; // 黄色中心圈表示未识别到
		Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
	}

	if (gimbal.visionEnable && visionFindAver >= 0.5f) // 自瞄开启并识别成功
	{
		Gimbal_VisionCtrl(); // 由视觉控制云台
	}
	else // 自瞄未开启或未识别到，由摇杆或鼠标控制
	{
		if (gimbal.rockerCtrl)
			Gimbal_RockerCtrl();
		else
			Gimbal_MouseCtrl();
	}

	// 更新陀螺仪角度
	Gimbal_UpdataAngle();
	// 计算yaw电机输出
	PID_CascadeCalc(&gimbal.yaw.imuPID, gimbal.yaw.targetAngle, gimbal.yaw.totalAngle, gimbal.yaw.gyro);
	// 计算pitch电输出
	PID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
	temp = gimbal.pitchMotor.angle / 100.0;
}

#ifdef EN_GIMBAL_TASK
void os_GimbalCallback(void const *argument)
{

	for (;;)
	{
		Task_Gimbal_Callback();
		osDelay(2);
	}
}
#endif
