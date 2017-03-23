#ifndef __ATM32ADC_H
#define __ATM32ADC_H

#include "stm32f4xx.h"



// 模数转换引出 的API 程序
extern void ADC_Voltage_initial(void); //初始化，在上电的时候调用一次。之后 ADC会自动采集更新
extern int16_t Get_Bat_Vol(void);  //读取当前的电池电压值， 单位 0.1V
extern int16_t Get_Sevor_Vol(void);	//读取当前的舵机电压值， 单位 0.1V

#endif

//------------------End of File----------------------------
