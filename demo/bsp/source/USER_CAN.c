/*************自定义CAN通信************/

#include "user_can.h"
#include "can.h"
#include "Moto.h"
#include "chassis.h"
#include "Shooter.h"
#include "Gimbal.h"
#include "Vision.h"
#include "Detect.h"
#include "super_cap.h"

uint32_t debugVisionInterval = 0;
CanState can_state;

/**************内部工具函数声明***********************/
void CAN1_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
// can2接收
void CAN2_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);

/******************初始化***************************/
// can过滤器初始化
void CAN_Init()
{
	CAN_FilterTypeDef can_filter;

	can_filter.FilterBank = 0;					   // filter 0
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK; // mask mode
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterIdHigh = 0;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow = 0;					// set mask 0 to receive all can id
	can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
	can_filter.FilterActivation = ENABLE;			// enable can filter

	can_state.can1_user_init_error += HAL_CAN_ConfigFilter(&hcan1, &can_filter);						 // init can filter
	can_state.can1_user_init_error += HAL_CAN_Start(&hcan1);											 // start can1
	can_state.can1_user_init_error += HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt
	// can_state.can1_user_init_error += HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	can_filter.SlaveStartFilterBank = 14;
	can_filter.FilterBank = 14;

	can_state.can2_user_init_error += HAL_CAN_ConfigFilter(&hcan2, &can_filter);
	can_state.can2_user_init_error += HAL_CAN_Start(&hcan2);
	can_state.can2_user_init_error += HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // 挂起中断允许
																										 // an_state.can2_user_init_error += HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
																										 //  英文注释都是大疆官方的电机驱动例程里面的
}
/*********************can接受回调函数*************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef if_can_get_message_ok;
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if (hcan == &hcan1)
	{
		if_can_get_message_ok = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if (if_can_get_message_ok == HAL_OK)
		{

			CAN1_Rx0Callback(&rx_header, rx_data);
		}
		else
		{
			can_state.can1_receive_error++;
		}
	}
	else if (hcan == &hcan2)
	{
		if_can_get_message_ok = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if (HAL_OK == if_can_get_message_ok)
		{
			CAN2_Rx0Callback(&rx_header, rx_data);
		}
		else
		{
			can_state.can2_receive_error++;
		}
	}
}

// can1接收结束中断
void CAN1_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
	uint8_t whichMotor;
	switch (rx_header->StdId)
	{
	// 底盘四个电机
	case 0x201:
	case 0x202:
	case 0x203:
	case 0x204:
	{
		whichMotor = rx_header->StdId - 0x201; // 判定来自哪个电机
		// 储存速度 角度 转矩电流 温度信息
		Motor_Update(&chassis.motors[whichMotor], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// 更新掉线检测数据
		Detect_Update(DeviceID_ChassisMotor1 + whichMotor);
	}
	break;
	// 拨弹电机
	case 0x205:
		Motor_Update(&shooter.triggerMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// 更新掉线检测数据
		Detect_Update(DeviceID_TrigMotor);
		break;
		// 云台yaw轴电机
	case 0x206:
		Motor_Update(&gimbal.yawMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// 更新掉线检测数据
		Detect_Update(DeviceID_YawMotor);
		break;
	case CAP_GET_IN_DATA_CANID:
		cap.receive_data.input_pow = ((rxdata[0] << 8) | rxdata[1]);
		cap.receive_data.input_vot = ((rxdata[2] << 8) | rxdata[3]);
		cap.receive_data.input_cur = ((rxdata[4] << 8) | rxdata[5]);
		break;
	case CAP_GET_OUT_DATA_CANID:
		cap.receive_data.output_pow = ((rxdata[0] << 8) | rxdata[1]);
		cap.receive_data.output_vot = ((rxdata[2] << 8) | rxdata[3]);
		cap.receive_data.output_cur = ((rxdata[4] << 8) | rxdata[5]);
		break;
	case CAP_STATE_ERROR_CANID:
		cap.receive_data.state = ((rxdata[0] << 8) | rxdata[1]);
		cap.receive_data.error = ((rxdata[2] << 8) | rxdata[3]);
		break;
		// 未知信息
	default:
		break;
	}
}
////can2接收结束中断
void CAN2_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
	uint8_t whichMotor;
	switch (rx_header->StdId)
	{
	// 摩擦轮电机
	case 0x203:
	case 0x204:
		whichMotor = rx_header->StdId - 0x203;
		Motor_Update(&shooter.fricMotor[whichMotor], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// 更新掉线检测数据
		Detect_Update(DeviceID_FricMotor1 + whichMotor);
		break;
	// 云台pitch电机
	case 0x207:
		Motor_Update(&gimbal.pitchMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// 更新掉线检测数据
		Detect_Update(DeviceID_PitchMotor);
		break;
	// 视觉数据
	case 0xAA:
		Vision_ParseData(rxdata);
		// 更新掉线检测数据
		Detect_Update(DeviceID_PC);
		// 计算两次收到视觉数据的间隔(debug用)
		static uint32_t lastTime = 0;
		debugVisionInterval = HAL_GetTick() - lastTime;
		lastTime = HAL_GetTick();
		break;
	// 未知信息
	default:
		break;
	}
}

/********************外部调用函数*******************************/
// 发送电机电流信息
void USER_CAN_SetMotorCurrent(CAN_HandleTypeDef *hcan, int16_t StdId, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	CAN_TxHeaderTypeDef tx_header;
	HAL_StatusTypeDef if_can_send_message_ok;
	uint8_t tx_data[8];

	tx_header.StdId = StdId;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	tx_data[0] = (iq1 >> 8) & 0xff;
	tx_data[1] = (iq1)&0xff;
	tx_data[2] = (iq2 >> 8) & 0xff;
	tx_data[3] = (iq2)&0xff;
	tx_data[4] = (iq3 >> 8) & 0xff;
	tx_data[5] = (iq3)&0xff;
	tx_data[6] = (iq4 >> 8) & 0xff;
	tx_data[7] = (iq4)&0xff;

	if_can_send_message_ok = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
	if (if_can_send_message_ok != HAL_OK)
	{
		if (hcan == &hcan1)
		{
			can_state.can1_send_error++;
		}
		else if (hcan == &hcan2)
		{
			can_state.can2_send_error++;
		}
	}
}

// 向视觉发送数据
void USER_CAN_SendVisionBuf(CAN_HandleTypeDef *hcan, uint8_t *buf, uint8_t len)
{
	CAN_TxHeaderTypeDef tx_header = {0};
	uint8_t tx_data[8];
	HAL_StatusTypeDef if_can_send_message_ok;

	tx_header.StdId = 0xAB;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	for (uint8_t i = 0; i < len; i++)
		tx_data[i] = buf[i];
	if_can_send_message_ok = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
	if (if_can_send_message_ok != HAL_OK)
	{
		if (hcan == &hcan1)
		{
			can_state.can1_send_error++;
		}
		else if (hcan == &hcan2)
		{
			can_state.can2_send_error++;
		}
	}
}

void USER_CAN_GetCapData(CAN_HandleTypeDef *hcan, int16_t StdId)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[4];
	tx_header.StdId = StdId;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_REMOTE;
	tx_header.DLC = 0x00;
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

void USER_CAN_SendCapData(CAN_HandleTypeDef *hcan, int16_t StdId, uint16_t data)
{

	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[4];

	tx_header.StdId = StdId;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 4;

	tx_data[0] = (data >> 8) & 0xff;
	tx_data[1] = (data)&0xff;
	tx_data[2] = 0x00;
	tx_data[3] = 0x01;

	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}
