#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "delay.h"

#define Red     0
#define Blue    1
#define Green   2

#define ON  0xff
#define OFF 0x00

//LEDs 定义LED 操作宏.
//(输出低电平,灯亮;输出高电平灯灭)

//xiang:我把绿灯和蓝灯都注释掉了，为了如果用了这两个函数，可以在编译时发现错误
#define LEDRed_ON()   GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define LEDRed_OFF()  GPIO_SetBits(GPIOA, GPIO_Pin_8)

//#define LEDGreen_ON()   //GPIO_ResetBits(GPIOA, GPIO_Pin_8)
//#define LEDGreen_OFF()  //GPIO_SetBits(GPIOA, GPIO_Pin_8)

//#define LEDBlue_ON()   GPIO_ResetBits(GPIOA, GPIO_Pin_8)
//#define LEDBlue_OFF()  GPIO_SetBits(GPIOA, GPIO_Pin_8)

//#define LEDALL_ON()   GPIO_ResetBits(GPIOA, GPIO_Pin_8)
//#define LEDALL_OFF()  GPIO_SetBits(GPIOA, GPIO_Pin_8)

//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0x02
#define SWD_ENABLE         0x01
#define JTAG_SWD_ENABLE    0x00	

void Initial_LED_GPIO(void);
void LED_Set_Blink(uint8_t LED, uint16_t on_time,uint16_t off_time,uint8_t repeat);
void LED_Blink_Routine(void);
#endif


//------------------End of File----------------------------
