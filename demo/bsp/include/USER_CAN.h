#ifndef _USER_CAN_H_
#define _USER_CAN_H_

#include "stdint.h"
#include "can.h"
#include "cmsis_os.h"
//can��״̬��Ϣ������debug
typedef struct
{
	HAL_StatusTypeDef can1_user_init_error,can2_user_init_error;
	uint16_t can1_send_error,can2_send_error;
	uint16_t can1_receive_error,can2_receive_error;
}CanState;

/****�ӿں�������****/
//���͵��������Ϣ
void USER_CAN_SetMotorCurrent(CAN_HandleTypeDef *hcan,int16_t StdId,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
//���Ӿ���������
void USER_CAN_SendVisionBuf(CAN_HandleTypeDef *hcan,uint8_t *buf,uint8_t len);

void USER_CAN_GetCapData(CAN_HandleTypeDef *hcan,int16_t StdId);
void USER_CAN_SendCapData(CAN_HandleTypeDef *hcan,int16_t StdId,uint16_t set_target_power);
//can��ʼ��
void CAN_Init(void);

#endif
