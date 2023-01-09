//
// Created by 刘昱杉2020 on 2023/1/8.
//
#include "tim.h"
#include "n20.h"
#include "PID.h"
//#define ABS(x) ((x)>0?(x):-(x))
Motor n20;
#define N20_EN


void N20_Init()
{

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
    
}
void N20_SetSpeed(Motor *motor,float speed)
{
    motor->TargetAngle = speed;
}
void N20_SetPWM(float PWM)
{
    if(PWM >0)
    {
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
    }
    else
    {
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,ABS(PWM));
    }
}
//统计马达的角度和速度
void Motor_Cal(Motor *motor)
{
    int32_t dAngle=0;
    if(motor->Angle-motor->LastAngle<-30000)
        dAngle=motor->Angle+(65535-motor->LastAngle);
    else if(motor->Angle-motor->LastAngle>30000)
        dAngle=-motor->LastAngle-(65535-motor->Angle);
    else
        dAngle=motor->Angle-motor->LastAngle;
    //将角度增量加入计数器
    motor->TotalAngle+=dAngle;
    //计算速度
    motor->Speed = (float)dAngle/(4*13*48)*50*60;//rpm   *50*60与定时器频率匹配
    //记录角度
    motor->LastAngle=motor->Angle;
}
void Motor_GetSpeed(Motor *motor)
{
    motor->Angle= __HAL_TIM_GET_COUNTER(&htim8);
    Motor_Cal(motor);
}
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim5)
        Motor_GetSpeed(&n20);
}
void N20_Callback()
{
    N20_SetSpeed(&n20,1000);
}
void os_N20TaskCallback(const void *argument)
{
#ifdef N20_EN
    N20_Callback();
#endif

}

