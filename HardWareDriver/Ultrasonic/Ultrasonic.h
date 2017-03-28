#ifndef __Ultrasonic_H
#define __Ultrasonic_H

#include "stm32f4xx.h"

extern float  Ultra_Distance;
extern uint8_t Ultra_ALT_Updated ; //高度更新完成标志。
extern int16_t Ultra_Healthy;

//超声波模块引出的API 程序
void Ultrasonic_initial(void); //初始化，上电的时候需要调用一次
void Ultrasonic_Routine(void); //超声波线程，需要放到循环中，不断调用。

#endif
