#include "UserFreertos.h"
#include "main.h"
#include "rc.h"
//#include "usb_device.h"
#include "tim.h"
#include "beep.h"
#include "user_can.h"
#include "user_imu.h"
// freertos测试程序 绿灯闪烁 上报任务占有率
/************仅供测试使用，在产品发布时应关闭**************/
/*
    相关宏定义configGENERATE_RUN_TIME_STATS
              configUSE_STATS_FORMATTING_FUNCTIONS
              configUSE_TRACE_FACILITY
    相关函数  configureTimerForRunTimeStats()
              getRunTimeCounterValue()
              vTaskListv((char *)&CPU_RunInfo)
              TaskGetRunTimeStats((char *)&CPU_RunInfo)
              HAL_TIM_PeriodElapsedCallback(&htim5)
    相关变量  RUN_Time
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
    HAL_GPIO_TogglePin(GPIOH, LED_BLUE_Pin); // 蓝灯闪烁
    osDelay(500);
    /**************上报任务占有情况******************/
    //    memset(CPU_RunInfo,0,400); //信息缓冲区清零
    //    vTaskList((char *)&CPU_RunInfo); //获取任务运行时间信息
    //    usb_printf("任务名   任务状态   优先级   剩余栈   任务序号\r\n");
    //    usb_printf("%s", CPU_RunInfo);
    //
    //
    //    memset(CPU_RunInfo,0,400); //信息缓冲区清零
    //    vTaskGetRunTimeStats((char *)&CPU_RunInfo);
    //    usb_printf("任务名         运行计数         使用率\r\n");
    //    usb_printf("%s", CPU_RunInfo);
    //    osDelay(500);
  }
}

// 错误处理(急停)任务
/*
 无os_delay最高优先级 在程序正常运行时不应该被调用
 在恢复执行时占据全部时间片饿死其他任务
*/
void os_ErrorCallback(void const *argument)
{
  osThreadSuspend(ErrorTaskHandle); // 第一次执行挂起自身
  for (;;)
  {
    USER_CAN_SetMotorCurrent(&hcan1, 0x200, 0, 0, 0, 0); // 关断电机
    USER_CAN_SetMotorCurrent(&hcan2, 0x200, 0, 0, 0, 0);
    USER_CAN_SetMotorCurrent(&hcan1, 0x1ff, 0, 0, 0, 0);
    USER_CAN_SetMotorCurrent(&hcan2, 0x1ff, 0, 0, 0, 0);
    if (rcInfo.right == 3)
      HAL_NVIC_SystemReset(); // 右拨杆回到中间重启系统
    // 红灯闪烁
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
    HAL_Delay(100);
  } // 无os_delay 最高优先级
}
