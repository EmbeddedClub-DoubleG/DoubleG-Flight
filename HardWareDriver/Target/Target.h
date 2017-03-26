#ifndef __Target_H
#define __Target_H

#include "PWM_Input.h"

extern float  Target_Roll,Target_Pitch,Target_Yaw,Tartget_hight;

#if Yingzhang_GCS
//上位机控制四轴目标角度相关变量
extern int16_t GCSControl_CH1;
extern int16_t GCSControl_CH2;
extern int16_t GCSControl_CH3;
extern int16_t GCSControl_CH4;
#endif


unsigned char Read_Mode(void);
void Get_Tartget_RPY(void);
void Lock_Target_Yaw(void);

#endif
