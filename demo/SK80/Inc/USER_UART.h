#ifndef _USER_UART_H_
#define _USER_UART_H_
#include "usart.h"
#include <stdbool.h>
#define UART1_RX_BUFFSIZE 128
#define UART1_TX_BUFFSIZE 128
typedef struct
{
	UART_HandleTypeDef huart;
	struct
	{
		uint16_t length; // �������ַ�������
		uint16_t RIndex; // ���ն�ָ��
		uint16_t WIndex; // ����дָ��

	} Rx;
	struct
	{
		uint16_t length; // �������ַ�������
		uint16_t RIndex; // ���Ͷ�ָ��
		uint16_t WIndex; // ����дָ��
		uint8_t Ready;	 // ���б�־
	} Tx;
} UART_Info;

extern UART_Info UART1;

/**
 * @brief ���ڻ�ȡ��������
 * @param[in] *byte
 * @param[out] true/false
 * @retval none
 */
bool UART_GetChar(UART_HandleTypeDef *huart, uint8_t *byte);

/**
 * @brief ���ڻ�ȡ���һ������
 * @param[in] *byte
 * @param[out] true/false
 * @retval none
 */
bool UART_GetLastChar(UART_HandleTypeDef *huart, uint8_t *byte);

/**
 * @brief ���ڻ�ȡ�ַ���
 * @param[in] *string
 * @param[out] true/false
 * @retval none
 */
bool UART_GetStr(UART_HandleTypeDef *huart, uint8_t *string);

/**
 * @brief ���ڻ�ȡ�������
 * @param[in] *datas
 * @param[out] true/false
 * @retval none
 */
bool UART_GetDatas(uint8_t *datas, uint16_t length);

/**
 * @brief ���ڷ��͵�������
 * @param[in] byte
 * @param[out] true/false
 * @retval none
 */
bool UART_PutChar(UART_HandleTypeDef *huart, uint8_t byte);

/**
 * @brief ������ڽ��ջ�����
 * @param[in] byte
 * @param[out] true/false
 * @retval none
 */
void UART_ClearRXBuff(UART_HandleTypeDef *huart);
#endif
