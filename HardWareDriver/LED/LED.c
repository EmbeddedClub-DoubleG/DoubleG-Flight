/* KEY.C file
STM32-SDK 开发板相关例程
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-02-28
测试： 本程序已在第七实验室的STM32-SDK上完成测试
功能：实现	Captian 的LED初始化和操作 API

---------硬件上的引脚连接:----------
LEDA -->  PB5  	(输出低电平,灯亮;输出高电平灯灭)
LEDB -->  PB4  	(输出低电平,灯亮;输出高电平灯灭)
LEDC -->  PB3  	(输出低电平,灯亮;输出高电平灯灭)
------------------------------------
 */

#include "LED.h"


 struct {
 uint8_t Status;    // LED当前状态  ON  或者是  OFF
 uint32_t ON_ms;	// LED 亮的时间，单位ms
 uint32_t OFF_ms;	// LED 灭的时间，单位ms
 uint8_t repeat;	// 闪烁重复次数
 uint32_t time;		// 上次动作时间
 }LED_Red,LED_Blue,LED_Green; //红  蓝  绿灯

/**************************实现函数********************************************
*函数原型:		void Initial_LED_GPIO(void)
*功　　能:		配置 LED 对应的端口为输出
*******************************************************************************/
void Initial_LED_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //使能GPIOA 的时钟,
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
  //配置PA8 为推挽输出  刷新频率为2Mhz  
  GPIOB->AFR[0] &= 0xff000fff;	//开启SWD
  GPIOB->AFR[0] |= 0x00033000;  //禁止JTAG接口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //应用配置到GPIOB 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

   //设置LED 端口输出高电平, 关灯.
   GPIO_SetBits(GPIOA, GPIO_Pin_8);	
}

// LED 设置闪烁子程序
// uint8_t LED          指定颜色的LED 可以是 Red,Blue,Green中的一种，视硬件支持
// uint16_t on_time     LED发亮的时间长度，单位ms 
// uint16_t off_time    LED灭的时间长度， 单位也是ms
// uint8_t repeat       LED闪烁重复的次数
void LED_Set_Blink(uint8_t LED, uint16_t on_time,uint16_t off_time,uint8_t repeat){
	if(LED == Red){
	LED_Red.ON_ms = on_time*1000;
	LED_Red.OFF_ms = off_time*1000;
	LED_Red.repeat = repeat;
	LEDRed_ON();
	LED_Red.time = micros();
	LED_Red.Status = ON;
	}
//	else if(LED == Blue){
//	LED_Blue.ON_ms = on_time*1000;
//	LED_Blue.OFF_ms = off_time*1000;
//	LED_Blue.repeat = repeat;
//	LEDBlue_ON();
//	LED_Blue.time = micros();
//	LED_Blue.Status = ON;
//	} 
//	else if(LED == Green){
//	LED_Green.ON_ms = on_time*1000;
//	LED_Green.OFF_ms = off_time*1000;
//	LED_Green.repeat = repeat;
//	LEDGreen_ON();
//	LED_Green.time = micros();
//	LED_Green.Status = ON;
//	}
}

//LED控制线程  需要在主程序中定时调用 以控制LED闪烁
void LED_Blink_Routine(void){
//	if((LED_Red.repeat ==0)&&(LED_Green.repeat ==0)&&(LED_Blue.repeat ==0))
//		return;

	if(LED_Red.repeat !=0)
	{
		if(LED_Red.OFF_ms==0){
			LED_Red.repeat = 0;
		}
		if(LED_Red.Status == ON)
		{
			if((micros()-LED_Red.time)>LED_Red.ON_ms){
			LEDRed_OFF();
			LED_Red.time = micros();
			LED_Red.Status = OFF;
			}	
		}
		else{
			if((micros()-LED_Red.time)>LED_Red.OFF_ms)
			{
				LED_Red.repeat--;
				if(LED_Red.repeat!=0){
				LEDRed_ON();
				LED_Red.time = micros();
				LED_Red.Status = ON;
				}
			}
		}
	}

//if(LED_Green.repeat !=0){
//  if(LED_Green.OFF_ms==0){
//  	LED_Green.repeat = 0;
//	}
//  if(LED_Green.Status == ON){
//  	if((micros()-LED_Green.time)>LED_Green.ON_ms){
//		LEDGreen_OFF();
//		LED_Green.time = micros();
//		LED_Green.Status = OFF;
//		}	
//  }else{
//  if((micros()-LED_Green.time)>LED_Green.OFF_ms){
//		LED_Green.repeat--;
//		if(LED_Green.repeat!=0){
//		LEDGreen_ON();
//		LED_Green.time = micros();
//		LED_Green.Status = ON;
//		}
//		}
//  }
//}

//if(LED_Blue.repeat !=0){
//  if(LED_Blue.OFF_ms==0){
//  	LED_Blue.repeat = 0;
//	}
//  if(LED_Blue.Status == ON){
//  	if((micros()-LED_Blue.time)>LED_Blue.ON_ms){
//		LEDBlue_OFF();
//		LED_Blue.time = micros();
//		LED_Blue.Status = OFF;
//		}	
//  }else{
//  if((micros()-LED_Blue.time)>LED_Blue.OFF_ms){
//		LED_Blue.repeat--;
//		if(LED_Blue.repeat!=0){
//		LEDBlue_ON();
//		LED_Blue.time = micros();
//		LED_Blue.Status = ON;
//		}
//		}
//  }
//}

}

//------------------End of File----------------------------
