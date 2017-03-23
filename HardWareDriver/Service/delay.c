/* delay.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-25
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用资源：
1. SYSTICK 模块  做为延时的计时

功能：
提供精确的延时API  有微秒级 和毫秒级延时
------------------------------------
 */
#include "delay.h"
	 
static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数

void Initial_System_Timer(void);

//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
/**************************实现函数********************************************
*函数原型:		void delay_init(u8 SYSCLK)
*功　　能:		初始化延迟系统，使延时程序进入可用状态
*******************************************************************************/
void delay_init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}				
				    
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
/**************************实现函数********************************************
*函数原型:		void delay_ms(u16 nms)
*功　　能:		毫秒级延时  延时nms  nms<=1864 
*******************************************************************************/
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
}   

//延时nus
//nus为要延时的us数.
/**************************实现函数********************************************
*函数原型:		void delay_us(u32 nus)
*功　　能:		微秒级延时  延时nus  nms<=1864 
*******************************************************************************/		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}

//初始化TIM5 32位定时器，用于做系统的时钟。 
void Initial_System_Timer(void){
    RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; //启动定时器
}

//------------------End of File----------------------------
