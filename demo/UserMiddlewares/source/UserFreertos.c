#include "UserFreertos.h"
#include "main.h"
#include "rc.h"
//#include "usb_device.h"
#include "tim.h"
#include "beep.h"
#include "user_can.h"
#include "user_imu.h"
// freertos���Գ��� �̵���˸ �ϱ�����ռ����
/************��������ʹ�ã��ڲ�Ʒ����ʱӦ�ر�**************/
/*
    ��غ궨��configGENERATE_RUN_TIME_STATS
              configUSE_STATS_FORMATTING_FUNCTIONS
              configUSE_TRACE_FACILITY
    ��غ���  configureTimerForRunTimeStats()
              getRunTimeCounterValue()
              vTaskListv((char *)&CPU_RunInfo)
              TaskGetRunTimeStats((char *)&CPU_RunInfo)
              HAL_TIM_PeriodElapsedCallback(&htim5)
    ��ر���  RUN_Time
              CPU_RunInfo
*/

// uint32_t RUN_Time=0;

// void configureTimerForRunTimeStats(void)
//{
//   RUN_Time=0;
//	HAL_TIM_Base_Start_IT(&htim5);
// }
// unsigned long getRunTimeCounterValue(void)
//{
// return RUN_Time;
// }

//  uint8_t CPU_RunInfo[400];
void os_LEDCallback(void const *argument)
{

  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOH, LED_BLUE_Pin); // ������˸
    osDelay(500);
    /**************�ϱ�����ռ�����******************/
    //    memset(CPU_RunInfo,0,400); //��Ϣ����������
    //    vTaskList((char *)&CPU_RunInfo); //��ȡ��������ʱ����Ϣ
    //    usb_printf("������   ����״̬   ���ȼ�   ʣ��ջ   �������\r\n");
    //    usb_printf("%s", CPU_RunInfo);
    //
    //
    //    memset(CPU_RunInfo,0,400); //��Ϣ����������
    //    vTaskGetRunTimeStats((char *)&CPU_RunInfo);
    //    usb_printf("������         ���м���         ʹ����\r\n");
    //    usb_printf("%s", CPU_RunInfo);
    //    osDelay(500);
  }
}

// ������(��ͣ)����
/*
 ��os_delay������ȼ� �ڳ�����������ʱ��Ӧ�ñ�����
 �ڻָ�ִ��ʱռ��ȫ��ʱ��Ƭ������������
*/
void os_ErrorCallback(void const *argument)
{
  osThreadSuspend(ErrorTaskHandle); // ��һ��ִ�й�������
  for (;;)
  {
    USER_CAN_SetMotorCurrent(&hcan1, 0x200, 0, 0, 0, 0); // �ضϵ��
    USER_CAN_SetMotorCurrent(&hcan2, 0x200, 0, 0, 0, 0);
    USER_CAN_SetMotorCurrent(&hcan1, 0x1ff, 0, 0, 0, 0);
    USER_CAN_SetMotorCurrent(&hcan2, 0x1ff, 0, 0, 0, 0);
    if (rcInfo.right == 3)
      HAL_NVIC_SystemReset(); // �Ҳ��˻ص��м�����ϵͳ
    // �����˸
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
    HAL_Delay(100);
  } // ��os_delay ������ȼ�
}
