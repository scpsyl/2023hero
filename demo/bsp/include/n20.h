//
// Created by 刘昱杉2020 on 2023/1/8.
//

#ifndef TEST1_N20_H
#define TEST1_N20_H
#include "PID.h"
#include "tim.h"
typedef  struct
{
    int16_t Angle;
    int16_t TargetSpeed;
    float Speed;
    int32_t TargetAngle;
    int32_t LastAngle;
    int32_t TotalAngle;
    singlePID   SpeedPID;
    CascadePID AnglePID;

}Motor;

void N20_Init(void);
void N20_SetSpeed(Motor *motor,float speed);
void N20_SetPWM(float PWM);
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void N20_Callback(void);

#endif //TEST1_N20_H
