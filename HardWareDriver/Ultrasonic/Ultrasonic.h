#ifndef __Ultrasonic_H
#define __Ultrasonic_H

#include "stm32f4xx.h"

//引出的距离  单位 米。
extern float  Ultra_Distance;
#define Max_Range	3.5f
// extern float Ultra_Debug_MaxDif;
extern float Ultra_Debug_Distance;
// extern uint32_t Ultra_dt;
//update20161227：注释掉三行
// #define Flag_Ultr 0x01
// #define Flag_MS5611 0x00
// extern uint8_t Flag_Ultr_or_MS5611;
//超声波模块引出的API 程序
void Ultrasonic_initial(void); //初始化，上电的时候需要调用一次
void Ultrasonic_Routine(void); //超声波线程，需要放到循环中，不断调用。
// float Ultrasonic_Get_D(void);//取超声波的距离变化率 变化率单位 米每秒//update20161227
#endif

//------------------End of File----------------------------







