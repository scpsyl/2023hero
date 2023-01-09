#ifndef _USER_LASER_H_
#define _USER_LASER_H

#include <stdint.h>
// Ĭ��ʹ��UART1��ΪSK80ͨ�Ŵ���
// ADDRʹ��Ĭ��ֵ0x80(��ģ��)
#define UART1_PutChar(a) UART_PutChar(&huart1, a)
#define disNum(a) ~(a) + 1 // ȡ����

// Function Declaration
/**
 * @brief ���ģ���ʼ��
 * @param[in] none
 * @param[out] none
 * @retval ������ʼ��Ϊ���� ����20m,Ƶ��20Hz,�ֱ���1mm,����������׼
 */
void Laser_Init(void);

/**
 * @brief ������ʼ��
 * @param[in] PositionStart
 * @param[out] none
 * @retval 0��β������ 1��ʼ������
 */
uint8_t LASER_SetPositionStart(uint8_t start);

/**
 * @brief ��������
 * @param[in] Range
 * @param[out] none
 * @retval 5m,10m,30m,50m,80m
 */
uint8_t Laser_SetRange(uint8_t range);

/**
 * @brief ����Ƶ��
 * @param[in] Frequency
 * @param[out] none
 * @retval 1(5Hz),2(10Hz),3(20Hz)
 */
uint8_t Laser_SetFreq(uint8_t freq);

/**
 * @brief ���÷ֱ���
 * @param[in] Solution
 * @param[out] none
 * @retval 1(1mm),2(0.1mm)
 */
uint8_t Laser_SetSolution(uint8_t solu);

/**
 * @brief �޸ľ���
 * @param[in] Singal Distance
 * @param[out] none
 * @retval signal:0x2BΪ��,0x2DΪ��
 * @example +23mm:FA 04 06 2B 17 BA
 */
uint8_t Laser_ChangeDistance(uint8_t signal, uint8_t dist);

/**
 * @brief �����ϵ缴��
 * @param[in] Start
 * @param[out] none
 * @retval 0(�ر�),1(����)
 */
uint8_t Laser_MeasuredOnPower(uint8_t start);

/**
 * @brief �Ƿ�򿪼�����׼
 * @param[in] Enable
 * @param[out] none
 * @retval 0(�ر�),1(����)
 */
uint8_t Laser_OpenLight(uint8_t enable);

/**
 * @brief ���β���
 * @param[in] none
 * @param[out] none
 * @retval
 */
void Laser_MeasureSignal(void);

/**
 * @brief ���β���-�㲥
 * @param[in] none
 * @param[out] none
 * @retval ͨ�����͵��β���-�㲥&��ȡ���� �ӿ����
 */
void Laser_MeasureBroadcast(void);

/**
 * @brief ��ȡ����
 * @param[in] none
 * @param[out] none
 * @retval ��ϵ��ι㲥ʹ�üӿ����
 */
void Laser_ReadCache(void);

/**
 * @brief ��������
 * @param[in] none
 * @param[out] none
 * @retval
 */
void Laser_MeasureContinous(void);

/**
 * @brief �������ݷ���ʱ����
 * @param[in] MeaInterver
 * @param[out] none
 * @retval ��������ģʽ�� ��λms
 */
void Laser_SetMeaInte(uint8_t Int);

/**
 * @brief ��ȡ��Ŀ��ʵ�ʾ���
 * @param[in] *Distance
 * @param[out] none
 * @retval
 */
uint8_t Laser_GetDistance(uint16_t *distance);

/**
 * @brief �ػ�
 * @param[in] none
 * @param[out] none
 * @retval
 */
void Laser_Close(void);

void SK80_Init(void);


#endif
