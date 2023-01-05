/*************�Զ���CANͨ��************/

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

/**************�ڲ����ߺ�������***********************/
void CAN1_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
// can2����
void CAN2_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);

/******************��ʼ��***************************/
// can��������ʼ��
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
	can_state.can2_user_init_error += HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // �����ж�����
																										 // an_state.can2_user_init_error += HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
																										 //  Ӣ��ע�Ͷ��Ǵ󽮹ٷ��ĵ���������������
}
/*********************can���ܻص�����*************************/
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

// can1���ս����ж�
void CAN1_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
	uint8_t whichMotor;
	switch (rx_header->StdId)
	{
	// �����ĸ����
	case 0x201:
	case 0x202:
	case 0x203:
	case 0x204:
	{
		whichMotor = rx_header->StdId - 0x201; // �ж������ĸ����
		// �����ٶ� �Ƕ� ת�ص��� �¶���Ϣ
		Motor_Update(&chassis.motors[whichMotor], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// ���µ��߼������
		Detect_Update(DeviceID_ChassisMotor1 + whichMotor);
	}
	break;
	// �������
	case 0x205:
		Motor_Update(&shooter.triggerMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// ���µ��߼������
		Detect_Update(DeviceID_TrigMotor);
		break;
		// ��̨yaw����
	case 0x206:
		Motor_Update(&gimbal.yawMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// ���µ��߼������
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
		// δ֪��Ϣ
	default:
		break;
	}
}
////can2���ս����ж�
void CAN2_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
	uint8_t whichMotor;
	switch (rx_header->StdId)
	{
	// Ħ���ֵ��
	case 0x203:
	case 0x204:
		whichMotor = rx_header->StdId - 0x203;
		Motor_Update(&shooter.fricMotor[whichMotor], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// ���µ��߼������
		Detect_Update(DeviceID_FricMotor1 + whichMotor);
		break;
	// ��̨pitch���
	case 0x207:
		Motor_Update(&gimbal.pitchMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
		// ���µ��߼������
		Detect_Update(DeviceID_PitchMotor);
		break;
	// �Ӿ�����
	case 0xAA:
		Vision_ParseData(rxdata);
		// ���µ��߼������
		Detect_Update(DeviceID_PC);
		// ���������յ��Ӿ����ݵļ��(debug��)
		static uint32_t lastTime = 0;
		debugVisionInterval = HAL_GetTick() - lastTime;
		lastTime = HAL_GetTick();
		break;
	// δ֪��Ϣ
	default:
		break;
	}
}

/********************�ⲿ���ú���*******************************/
// ���͵��������Ϣ
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

// ���Ӿ���������
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
