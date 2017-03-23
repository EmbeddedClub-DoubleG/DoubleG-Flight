#ifndef __PWM_Output_H
#define __PWM_Output_H

#include "stm32f4xx.h"
#include "LED.h"


//---------------PWM周期设置--------------------------
#define  PWM_1_2_3_4_Period 	 2500	  //PWM输出通道1.2.3.4的周期，单位 us  TIM5
#define  PWM_5_6_Period          3000	  //PWM输出通道5.6的周期，单位 us	   TIM3
#define  PWM_Hz                 (1000000/PWM_1_2_3_4_Period)

//----------------------------------------------------
#define  Servo_Period            3000	  //PWM输出的周期，单位 us
#define  Servo_Neutral_position   1500	  //中立值

//各个通道的默认值 单位 us
#define  PWMOuput_CH1_Default   1000//xiang：这里是我改的，本来是1000，为了安全，我改成了800
#define  PWMOuput_CH2_Default   1000
#define  PWMOuput_CH3_Default   1000
#define  PWMOuput_CH4_Default   1000
#define  PWMOuput_CH5_Default   1000
#define  PWMOuput_CH6_Default   1000

//-------------PWM 输出 API ------------------------------
// x 的单位是us.
#define  Set_PWMOuput_CH1(x)  TIM2->CCR4 = x ;\
							  PWM_Output_CH1 =x
							    
#define  Set_PWMOuput_CH2(x)  TIM2->CCR3 = x ;\
							  PWM_Output_CH2 =x

#define  Set_PWMOuput_CH3(x)  TIM2->CCR2 = x ;\
							  PWM_Output_CH3 =x

#define  Set_PWMOuput_CH4(x)  TIM2->CCR1 = x ;\
							  PWM_Output_CH4 =x

#define  Set_PWMOuput_CH5(x)  TIM4->CCR3 = x ;\
							  PWM_Output_CH5 =x

#define  Set_PWMOuput_CH6(x)  TIM4->CCR4 = x ;\
							  PWM_Output_CH6 =x

//------------↑PWM 输出 API ↑-----------------------------
extern volatile uint8_t THROTTLE_LOCKed;
extern volatile uint8_t  Fly_Mode;

extern volatile uint8_t Servo_Update_Req; //PWM输出更新请求 由定时器溢出中断置位
extern volatile uint16_t PWM_Output_CH1,PWM_Output_CH2,PWM_Output_CH3,PWM_Output_CH4,
					PWM_Output_CH5,PWM_Output_CH6,PWM_Output_CH7,PWM_Output_CH8;

void PWM_Output_Initial(void);	//初始化 
void PWM_Output_Set_default(void);	// 输出默认值
void PWM_Write_Motors(void);
#endif

//------------------End of File----------------------------
