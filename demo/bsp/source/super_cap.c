#include "super_cap.h"
#include "user_can.h"
#include "moto.h"
#include "UserFreertos.h"
#include "graphics.h"
#include "chassis.h"
#include "judge.h"
#include <stdio.h>
SuperCap cap;

void Cap_AnalysisData()
{
	uint16_t PowerBuffer;
	uint8_t PowerLimit;
	cap.output_vot = cap.receive_data.output_vot / 100.f;
	cap.input_pow = cap.receive_data.input_pow / 100.f;

	cap.energy = ABS((cap.output_vot * cap.output_vot - 15.f * 15.f) * 2.5f); // 电容能量 1/2CU^2 C=5F 以15v为最低电压
	cap.per_energy = cap.energy / 877.5f * 100.0f;							  // 相对15v的百分比能量值 877.5=0.5*5*（24*24-15*15）

	PowerBuffer = JUDGE_GetPowerBuffer();
	PowerLimit = JUDGE_GetChassisPowerLimit();

	if (cap.target_input_pow > cap.input_pow + 30)
	{
		cap.target_input_pow = cap.input_pow + 30;
	}
	else
	{
		if (PowerBuffer < 40)
		{
			cap.target_input_pow -= 1.7f;
		}
		else
		{
			cap.target_input_pow++;
		}
	}

	cap.target_input_pow = LIMIT(cap.target_input_pow, PowerLimit - 10, PowerLimit);
	cap.set_target_power = cap.target_input_pow * 100;
}

void os_SuperCapCallback(void const *argument)
{
	osDelay(5000);
	for (;;)
	{
		USER_CAN_GetCapData(&hcan1, CAP_GET_IN_DATA_CANID);
		osDelay(2);
		USER_CAN_GetCapData(&hcan1, CAP_GET_OUT_DATA_CANID);
		osDelay(2);
		Cap_AnalysisData();
		USER_CAN_SendCapData(&hcan1, CAP_SET_IN_POWER_CANID, cap.set_target_power);
		char textBuf[30] = {0};
		sprintf(textBuf, "CAP ENERGY= %03d %%", (int)cap.per_energy);
		Graph_SetText(&chassis.ui.super_cap, "CAP", Color_Orange, 2, 0, 100, 650, textBuf, 17, 20);
		Graph_DrawText(&chassis.ui.super_cap, Operation_Change);
		osDelay(2);
	}
}
