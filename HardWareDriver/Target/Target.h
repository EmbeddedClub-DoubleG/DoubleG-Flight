#ifndef __Target_H
#define __Target_H

#include "stm32f4xx.h"

extern float  Target_Roll,Target_Pitch,Target_Yaw;
extern volatile float Quad_THR;
extern int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;

#if Yingzhang_GCS
//上位机控制四轴目标角度相关变量
extern int16_t GCSControl_CH1;
extern int16_t GCSControl_CH2;
extern float GCSControl_CH3;
extern float GCSControl_CH3_Accumulate;
extern int16_t GCSControl_CH4;
#endif


unsigned char Read_Mode(void);
void Get_Tartget_RPY(void);
void Lock_Target_Yaw(void);
void Get_Throttle(void);
#endif
