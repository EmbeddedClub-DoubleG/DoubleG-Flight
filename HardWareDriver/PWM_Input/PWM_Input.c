/* PWM_Input.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-25
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用资源：
1. PWM 输入引脚
	PC0 PC1 PC2 PC3 PC4 PC5
2. 引脚变化外部中断线    
3. 读取系统时间API

功能：
配置PWM 输入，通过中断捕捉PWM输入的高电平时间。
------------------------------------
 */
#include "delay.h"
#include "PWM_Input.h"
//#include "LED.h"

//各个通道的上升沿和下降沿的 触发时间
volatile struct PWM_Input_struct1{ 
  uint32_t RisingTime; // 上升沿 时间 us
  uint32_t FallingTime; //下降沿 时间 us 
}CH1,CH2,CH3,CH4,CH5,CH6;

//当前 PWM输入的值  单位 us 
volatile int16_t PWM_Input_CH1,PWM_Input_CH2,PWM_Input_CH3,PWM_Input_CH4,PWM_Input_CH5,PWM_Input_CH6;
volatile int16_t RC_data[MAX_RC_Ch];

extern int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;

//配置 PWM输入通道 的中断优先级
static void PWM_Input_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		static void PWM_Input_GPIO_Config(void)
*功　　能:	    开启中断。并配置各通道的输入引脚为中断输入
*******************************************************************************/
static void PWM_Input_GPIO_Config(void) {
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/* config the extiline(PB0) clock and AFIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
	/* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* config the NVIC(PC0-5) */
	PWM_Input_NVIC_Config();

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;       
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	 // 上拉输入
  	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* EXTI line(PC0) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource0); 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//中断模式
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //开启上升和下降沿中断
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE; //使能中断
  	EXTI_Init(&EXTI_InitStructure); 
	
	/* EXTI line(PC1) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource1); 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 

	/* EXTI line(PC2) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource2); 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 
	
	/* EXTI line(PC3) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource3); 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 
	
	/* EXTI line(PC4) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource4); 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 
	
	/* EXTI line(PC5) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource5); 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 
	
}

/**************************实现函数********************************************
*函数原型:		void PWM_Input_Initial(void)
*功　　能:	    开启中断。并配置各通道的输入引脚为中断输入
*******************************************************************************/
void PWM_Input_Initial(void)
{
	PWM_Input_GPIO_Config();
}

/**************************实现函数********************************************
*函数原型:		void EXTI0_IRQHandler(void)
*功　　能:	    中断线0 的中断程序
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
	uint32_t pwmin_temp;
	if( GPIOC->IDR & 0x0001){ //是上升沿？
		CH6.RisingTime = micros();	//取上升沿发生时的时间 T1
	}else{ //下降沿
		CH6.FallingTime = micros(); //取时间 T2
		if(CH6.FallingTime < CH6.RisingTime){
			return ; //超时 返回
		} else{
			pwmin_temp = CH6.FallingTime - CH6.RisingTime;//T2-T1
		}
		RC_data[5]	= PWM_Input_CH6;
		if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
		{
			PWM_Input_CH6 = pwmin_temp;
		}
	}
	EXTI->PR=1<<0;  //清除中断标志位  
}

//中断线1 的中断程序
void EXTI1_IRQHandler(void)
{
//TODO: 在接收不到遥控器信号的时候，不应该保持遥控器输入为前一次的输入，应该进入紧急降落模式（使用gps回到home地点并降落）
	uint32_t pwmin_temp;
	if( GPIOC->IDR & 0x0002){ 
		CH5.RisingTime = micros();
	}else{
		CH5.FallingTime = micros();
		if(CH5.FallingTime < CH5.RisingTime){
			//pwmin_temp = (0xffffffff - CH5.RisingTime) + CH5.FallingTime;
			return ;
		} else{
			pwmin_temp = CH5.FallingTime - CH5.RisingTime;
		}
		RC_data[4]	= PWM_Input_CH5;
		if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
		{
			PWM_Input_CH5 = pwmin_temp;
		}
		//Set_PWMOuput_CH5(PWM_Input_CH5);
	}
	EXTI->PR=1<<1;  //清除中断标志位  
}

//中断线2 的中断程序
void EXTI2_IRQHandler(void)
{
	uint32_t pwmin_temp;
	if( GPIOC->IDR & 0x0004){ 
		CH4.RisingTime = micros();
	}else{
		CH4.FallingTime = micros();
		if(CH4.FallingTime < CH4.RisingTime){
			//pwmin_temp = (0xffffffff - CH4.RisingTime) + CH4.FallingTime;
			return ;
		} else{
			pwmin_temp = CH4.FallingTime - CH4.RisingTime;
		}
		RC_data[3]	= PWM_Input_CH4;
		if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
		{
			PWM_Input_CH4 = pwmin_temp - PWM_Offset_Yaw + 1500;
		}
		else{
			PWM_Input_CH4 = 1500;
		}
		//Set_PWMOuput_CH4(PWM_Input_CH4);
	}
	EXTI->PR=1<<2;  //清除中断标志位  
}

//中断线3 的中断程序
void EXTI3_IRQHandler(void)
{
	uint32_t pwmin_temp;
	if( GPIOC->IDR & 0x0008){ 
		CH3.RisingTime = micros();
	}else{
		CH3.FallingTime = micros();
		if(CH3.FallingTime < CH3.RisingTime){
			//pwmin_temp = (0xffffffff - CH3.RisingTime) + CH3.FallingTime;
			return ;
		} else{
			pwmin_temp = CH3.FallingTime - CH3.RisingTime;
		}
		RC_data[2]	= PWM_Input_CH3;
		if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
		{
			PWM_Input_CH3 = pwmin_temp;
		}
		//Set_PWMOuput_CH3(PWM_Input_CH3);
	}
	EXTI->PR=1<<3;  //清除中断标志位  
}

//中断线4 的中断程序
void EXTI4_IRQHandler(void)
{
	uint32_t pwmin_temp;
	if( GPIOC->IDR & 0x0010){ 
		CH2.RisingTime = micros();
	}else{
		CH2.FallingTime = micros();
		if(CH2.FallingTime < CH2.RisingTime){
			//pwmin_temp = (0xffffffff - CH2.RisingTime) + CH2.FallingTime;
			return ;
		} else{
			pwmin_temp = CH2.FallingTime - CH2.RisingTime;
		}
		RC_data[1]	= PWM_Input_CH2;
		if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
		{
			PWM_Input_CH2 = pwmin_temp - PWM_Offset_Pitch + 1500;
		}
		else{
		    PWM_Input_CH2 = 1500;
		}
		//Set_PWMOuput_CH2(PWM_Input_CH2);
	}
	EXTI->PR=1<<4;  //清除中断标志位  
}

//中断线9-5 的中断程序
unsigned char PPM_Enable = 0 ,PPM_Initial = 0;
unsigned char PPM_CH_Index = 0;
uint32_t      PPM_RisingTime_Last ;
int16_t		  PPM_CH_Value[20];
void EXTI9_5_IRQHandler(void)
{
	uint32_t pwmin_temp;   
	if(EXTI_GetITStatus(EXTI_Line5) == SET){ 
	if( GPIOC->IDR & 0x0020){  //上升沿产生的中断
		CH1.RisingTime = micros();
	}else{
		CH1.FallingTime = micros();
		if(CH1.FallingTime < CH1.RisingTime){
			//pwmin_temp = (0xffffffff - CH1.RisingTime) + CH1.FallingTime;
			return ;
		} else{
			pwmin_temp = CH1.FallingTime - CH1.RisingTime;
		}
		RC_data[0]	= PWM_Input_CH1;
		if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
		{
			PWM_Input_CH1 = pwmin_temp - PWM_Offset_Roll + 1500;
		}
		else{
		    PWM_Input_CH1 = 1500;
		}
	}
	EXTI->PR=1<<5;  //清除中断标志位 
	} 
}

//------------------End of File----------------------------
