#ifndef __PWM_Input_H
#define __PWM_Input_H

#include "stm32f4xx.h"

#define MAX_RC_Ch	6
#define PWM_Input_Offset  1500	 //中立值 1500 us
#define PWM_Input_MID   1500
#define PWM_Input_MAX   2200
#define PWM_Input_MIN   800
#define PWM_Input_Low_position  1000  //最低值 1000us

extern volatile uint16_t sysytem_time_ms;
//当前各个通道的PWM输入值，单位 us  供外部调用
extern volatile int16_t PWM_Input_CH1,PWM_Input_CH2,PWM_Input_CH3,
									PWM_Input_CH4,PWM_Input_CH5,PWM_Input_CH6;
extern volatile int16_t RC_data[MAX_RC_Ch];

/*
void PWM_Input_Initial(void); 开启PWM 输入捕捉
用户只需要调用这个程序 就完成了PWM输入的配置。接下来读取	
PWM_Input_CH1..6 就可以得到	相应的输入脉宽值。
*/
void PWM_Input_Initial(void);

#endif

//------------------End of File----------------------------
