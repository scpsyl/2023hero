/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IMUTaskHandle;
osThreadId LEDTaskHandle;
osThreadId MotorTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId RCTaskHandle;
osThreadId ErrorTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId ShooterTaskHandle;
osThreadId VisionTaskHandle;
osThreadId JudgeTaskHandle;
osThreadId DetectTaskHandle;
osThreadId BeepTaskHandle;
osThreadId SuperCapTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void os_IMUCallback(void const * argument);
void os_LEDCallback(void const * argument);
void os_MotorCallback(void const * argument);
void os_ChassisCallback(void const * argument);
void os_RCCallback(void const * argument);
void os_ErrorCallback(void const * argument);
void os_GimbalCallback(void const * argument);
void os_ShooterCallback(void const * argument);
void os_VisionCallback(void const * argument);
void os_JudgeCallback(void const * argument);
void os_DetectCallback(void const * argument);
void os_BeepCallback(void const * argument);
void os_SuperCapCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, os_IMUCallback, osPriorityAboveNormal, 0, 384);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, os_LEDCallback, osPriorityBelowNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, os_MotorCallback, osPriorityNormal, 0, 256);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, os_ChassisCallback, osPriorityNormal, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of RCTask */
  osThreadDef(RCTask, os_RCCallback, osPriorityAboveNormal, 0, 256);
  RCTaskHandle = osThreadCreate(osThread(RCTask), NULL);

  /* definition and creation of ErrorTask */
  osThreadDef(ErrorTask, os_ErrorCallback, osPriorityRealtime, 0, 128);
  ErrorTaskHandle = osThreadCreate(osThread(ErrorTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, os_GimbalCallback, osPriorityNormal, 0, 256);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of ShooterTask */
  osThreadDef(ShooterTask, os_ShooterCallback, osPriorityNormal, 0, 128);
  ShooterTaskHandle = osThreadCreate(osThread(ShooterTask), NULL);

  /* definition and creation of VisionTask */
  osThreadDef(VisionTask, os_VisionCallback, osPriorityNormal, 0, 256);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  /* definition and creation of JudgeTask */
  osThreadDef(JudgeTask, os_JudgeCallback, osPriorityNormal, 0, 256);
  JudgeTaskHandle = osThreadCreate(osThread(JudgeTask), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, os_DetectCallback, osPriorityNormal, 0, 128);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* definition and creation of BeepTask */
  osThreadDef(BeepTask, os_BeepCallback, osPriorityNormal, 0, 128);
  BeepTaskHandle = osThreadCreate(osThread(BeepTask), NULL);

  /* definition and creation of SuperCapTask */
  osThreadDef(SuperCapTask, os_SuperCapCallback, osPriorityNormal, 0, 256);
  SuperCapTaskHandle = osThreadCreate(osThread(SuperCapTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_os_IMUCallback */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_IMUCallback */
__weak void os_IMUCallback(void const * argument)
{
  /* USER CODE BEGIN os_IMUCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_IMUCallback */
}

/* USER CODE BEGIN Header_os_LEDCallback */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_LEDCallback */
__weak void os_LEDCallback(void const * argument)
{
  /* USER CODE BEGIN os_LEDCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_LEDCallback */
}

/* USER CODE BEGIN Header_os_MotorCallback */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_MotorCallback */
__weak void os_MotorCallback(void const * argument)
{
  /* USER CODE BEGIN os_MotorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_MotorCallback */
}

/* USER CODE BEGIN Header_os_ChassisCallback */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_ChassisCallback */
__weak void os_ChassisCallback(void const * argument)
{
  /* USER CODE BEGIN os_ChassisCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_ChassisCallback */
}

/* USER CODE BEGIN Header_os_RCCallback */
/**
* @brief Function implementing the RCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_RCCallback */
__weak void os_RCCallback(void const * argument)
{
  /* USER CODE BEGIN os_RCCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_RCCallback */
}

/* USER CODE BEGIN Header_os_ErrorCallback */
/**
* @brief Function implementing the ErrorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_ErrorCallback */
__weak void os_ErrorCallback(void const * argument)
{
  /* USER CODE BEGIN os_ErrorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_ErrorCallback */
}

/* USER CODE BEGIN Header_os_GimbalCallback */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_GimbalCallback */
__weak void os_GimbalCallback(void const * argument)
{
  /* USER CODE BEGIN os_GimbalCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_GimbalCallback */
}

/* USER CODE BEGIN Header_os_ShooterCallback */
/**
* @brief Function implementing the ShooterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_ShooterCallback */
__weak void os_ShooterCallback(void const * argument)
{
  /* USER CODE BEGIN os_ShooterCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_ShooterCallback */
}

/* USER CODE BEGIN Header_os_VisionCallback */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_VisionCallback */
__weak void os_VisionCallback(void const * argument)
{
  /* USER CODE BEGIN os_VisionCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_VisionCallback */
}

/* USER CODE BEGIN Header_os_JudgeCallback */
/**
* @brief Function implementing the JudgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_JudgeCallback */
__weak void os_JudgeCallback(void const * argument)
{
  /* USER CODE BEGIN os_JudgeCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_JudgeCallback */
}

/* USER CODE BEGIN Header_os_DetectCallback */
/**
* @brief Function implementing the DetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_DetectCallback */
__weak void os_DetectCallback(void const * argument)
{
  /* USER CODE BEGIN os_DetectCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_DetectCallback */
}

/* USER CODE BEGIN Header_os_BeepCallback */
/**
* @brief Function implementing the BeepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_BeepCallback */
__weak void os_BeepCallback(void const * argument)
{
  /* USER CODE BEGIN os_BeepCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_BeepCallback */
}

/* USER CODE BEGIN Header_os_SuperCapCallback */
/**
* @brief Function implementing the SuperCapTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_os_SuperCapCallback */
__weak void os_SuperCapCallback(void const * argument)
{
  /* USER CODE BEGIN os_SuperCapCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END os_SuperCapCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
