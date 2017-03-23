/* PWM_Output.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-25
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用STM32 资源：
1. 使用Tim3 Tim4 Tim5 定时器 产生PWM信号
2. 以下引脚
   PA0 PA1 PA2 PA3 
   PB8  PB9 
   PC6  PC7 
   引脚 用于产生PWM信号

功能：
初始化定时器 产生PWM信号
------------------------------------
 */

#include "PWM_Output.h"
#include "Quadrotor.h"
#include "fly_config.h"
#include "Fmath.h"

volatile uint8_t THROTTLE_LOCKed = 1; //电机锁定，不启动
//舵机信号 更新请求，在每个PWM周期中置1，通过主程序判断该值来决定是否更新舵机输出，
volatile uint8_t Servo_Update_Req=0;
//各个通道的当前输出值
volatile uint16_t PWM_Output_CH1,PWM_Output_CH2,PWM_Output_CH3,PWM_Output_CH4,
					PWM_Output_CH5,PWM_Output_CH6,PWM_Output_CH7,PWM_Output_CH8;


/**************************实现函数********************************************
*函数原型:		static void PWMOutput_GPIO_Config(void) 
*功　　能:	    配置PWM引脚为输出，并开启相应的时钟信号 和引脚复用时钟信号  	 
*******************************************************************************/
static void PWMOutput_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE); 

  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;		   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
  	
}

/**************************实现函数********************************************
*函数原型:		static void Tim3_NVIC_Config(void)
*功　　能:	    配置定时器3 的中断优先级别 	 
*******************************************************************************/
static void Tim2_NVIC_Config(void) {
 NVIC_InitTypeDef NVIC_InitStructure; 
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   //选择TIM2中断
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;   //
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能
 NVIC_Init(&NVIC_InitStructure); 
}

/**************************实现函数********************************************
*函数原型:		void TIM2_IRQHandler(void)
*功　　能:	    定时器中断程序 
*******************************************************************************/
void TIM2_IRQHandler(void)//定时器中断函数
{
  Servo_Update_Req = 1;	//请求舵机信号更新 告诉主程序 可以更新PWM输出了
  TIM2->SR &= 0xfffe;  //清中断标志
}

/**************************实现函数********************************************
*函数原型:		static void PWMOutput_TIMERs_Config(void)
*功　　能:	    配置PWM的对应定时器 
*******************************************************************************/
static void PWMOutput_TIMERs_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	
  Tim2_NVIC_Config();
	/* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = Servo_Period;      //PWM周期
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1;	    //设置预分频：即为1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseStructure.TIM_Period = PWM_5_6_Period;      //PWM周期
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = PWM_1_2_3_4_Period;  //PWM周期
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM2->DIER |= 0x0001;  //使能中断	 使能定时器2的 溢出中断

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = Servo_Neutral_position;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //使能通道1
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //使能通道2
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //使能通道3
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //使能通道3
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//使能通道4
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//使能通道4
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // 使能TIM4重载寄存器ARR
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM5重载寄存器ARR

  TIM_Cmd(TIM4, ENABLE);                   //使能定时器4
  TIM_Cmd(TIM2, ENABLE);                   //使能定时器5
}

/**************************实现函数********************************************
*函数原型:		void PWM_Output_Initial(void)
*功　　能:	    PWM输出 初始化。
*******************************************************************************/
void PWM_Output_Initial(void)
{
	PWMOutput_GPIO_Config();
	PWMOutput_TIMERs_Config();
	Set_PWMOuput_CH1(PWMOuput_CH1_Default);	 // 放默认值输出
	Set_PWMOuput_CH2(PWMOuput_CH2_Default);
	Set_PWMOuput_CH3(PWMOuput_CH3_Default);
	Set_PWMOuput_CH4(PWMOuput_CH4_Default);
	Set_PWMOuput_CH5(PWMOuput_CH5_Default);
	Set_PWMOuput_CH6(PWMOuput_CH6_Default);
}

/**************************实现函数********************************************
*函数原型:		void PWM_Output_Set_default(void)
*功　　能:	    PWM输出 默认值
*******************************************************************************/
void PWM_Output_Set_default(void) {

	Set_PWMOuput_CH1(PWMOuput_CH1_Default);
	Set_PWMOuput_CH2(PWMOuput_CH2_Default);
	Set_PWMOuput_CH3(PWMOuput_CH3_Default);
	Set_PWMOuput_CH4(PWMOuput_CH4_Default);
	Set_PWMOuput_CH5(PWMOuput_CH5_Default);
	Set_PWMOuput_CH6(PWMOuput_CH6_Default);

}

#define Limit_Max     (int16_t)20   //每秒约计算 600 次 
int16_t Last_PID_ROLL,Last_PID_PITCH,Last_PID_YAW;

// 将PID计算的值进行 平滑处理。动作不要太生猛了
void PWM_PID_Smooth(void)
{
   PID_ROLL = Last_PID_ROLL + Math_Constrain((PID_ROLL - Last_PID_ROLL),-Limit_Max,+Limit_Max);
   PID_PITCH = Last_PID_PITCH + Math_Constrain((PID_PITCH - Last_PID_PITCH),-Limit_Max,+Limit_Max);
   PID_YAW = Last_PID_YAW + Math_Constrain((PID_YAW - Last_PID_YAW),-Limit_Max,+Limit_Max);
   
   //将这一次的值保存，以便下次计算时使用
   Last_PID_ROLL = PID_ROLL;
   Last_PID_PITCH = PID_PITCH;
   Last_PID_YAW	= PID_YAW;
}

//将 四轴计算出来的控制量进行 混合输出
//根据不同的飞行器，使用不同的混合方法
#define PIDMIX(X,Y,Z) THROTTLE + PID_ROLL*X + PID_PITCH*Y + Yaw_DIRECT * PID_YAW*Z
#define Min_PWM_Out  800    //us
#define Max_PWM_Out  2200    //us
int16_t motor[8] = {1000,1000,1000,1000,1000,1000,1000,1000};
void PWM_Write_Motors(void)
{

#if (Remote_Config_Enable == 1)	 //条件编译，Remote_Config_Enable的定义见fly_config.h
	if(Quadrotor_Mode == Quad_ESC_Cal)return;	//正在校准电调，别添乱了	

	if(THROTTLE_LOCKed){	  //油门锁定中，输出PWM不让电机启动，以免伤人。
		LED_Set_Blink(Red,1000,1000,1); //绿色LED长亮 表示PWM锁定中
		//xiang:这两行代码是我注释掉的，为了实现下面说的功能
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		//return;
		//xiang：这段代码是我加的，我怕下面写的代码有问题，为了安全，直接在这里给PWM_Output_CHx赋值，只适用于十字型和x型四轴
		switch(Fly_Mode){
			case QUADP :	//十字型的四轴飞行器 
				motor[0] = Math_Constrain(PIDMIX( 0,+1,-1),Min_PWM_Out,Max_PWM_Out); //REAR	 后尾电机
				motor[1] = Math_Constrain(PIDMIX(-1, 0,+1),Min_PWM_Out,Max_PWM_Out); //RIGHT 右边电机
				motor[2] = Math_Constrain(PIDMIX(+1, 0,+1),Min_PWM_Out,Max_PWM_Out); //LEFT	 左边电机
				motor[3] = Math_Constrain(PIDMIX( 0,-1,-1),Min_PWM_Out,Max_PWM_Out); //FRONT 前面电机
				PWM_Output_CH1 =motor[0];
				PWM_Output_CH2 =motor[1];
				PWM_Output_CH3 =motor[2];
				PWM_Output_CH4 =motor[3];
				return;
			case QUADX :	//X 型的四轴飞行器
				motor[0] = Math_Constrain(PIDMIX(-1,+1,-1),Min_PWM_Out,Max_PWM_Out); //REAR_R  后右电机
				motor[1] = Math_Constrain(PIDMIX(-1,-1,+1),Min_PWM_Out,Max_PWM_Out); //FRONT_R 前右电机
				motor[2] = Math_Constrain(PIDMIX(+1,+1,+1),Min_PWM_Out,Max_PWM_Out); //REAR_L  后左电机
				motor[3] = Math_Constrain(PIDMIX(+1,-1,-1),Min_PWM_Out,Max_PWM_Out); //FRONT_L 前左电机
				PWM_Output_CH1 =motor[0];
				PWM_Output_CH2 =motor[1];
				PWM_Output_CH3 =motor[2];
				PWM_Output_CH4 =motor[3];
				return;
			default:
				return;
		}
	}
#endif

  PWM_PID_Smooth();  //平滑输出 

  //保护措施，当油门低时，不启动电机。	
  if(THROTTLE < (int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/20)){
  PID_ROLL = 0;	//油门量小于 5%  不启动电机
  PID_PITCH = 0; //所有的控制量清零，以防电机启动
  PID_YAW = 0;
  }

switch(Fly_Mode){
  case QUADP :	//十字型的四轴飞行器
    motor[0] = Math_Constrain(PIDMIX( 0,+1,-1),Min_PWM_Out,Max_PWM_Out); //REAR	 后尾电机
    motor[1] = Math_Constrain(PIDMIX(-1, 0,+1),Min_PWM_Out,Max_PWM_Out); //RIGHT 右边电机
    motor[2] = Math_Constrain(PIDMIX(+1, 0,+1),Min_PWM_Out,Max_PWM_Out); //LEFT	 左边电机
    motor[3] = Math_Constrain(PIDMIX( 0,-1,-1),Min_PWM_Out,Max_PWM_Out); //FRONT 前面电机
	//xiang：这段代码是我改的，目的是在锁定油门后，依然可以在上位机上看到pwm输出
	if(THROTTLE_LOCKed)
	{
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		PWM_Output_CH1 =motor[0];
		PWM_Output_CH2 =motor[1];
		PWM_Output_CH3 =motor[2];
		PWM_Output_CH4 =motor[3];
		return;
	}else{
		Set_PWMOuput_CH1(motor[0]);
		Set_PWMOuput_CH2(motor[1]);
		Set_PWMOuput_CH3(motor[2]);
		Set_PWMOuput_CH4(motor[3]);
	}
    break;
  case QUADX :	//X 型的四轴飞行器
    motor[0] = Math_Constrain(PIDMIX(-1,+1,-1),Min_PWM_Out,Max_PWM_Out); //REAR_R  后右电机
    motor[1] = Math_Constrain(PIDMIX(-1,-1,+1),Min_PWM_Out,Max_PWM_Out); //FRONT_R 前右电机
    motor[2] = Math_Constrain(PIDMIX(+1,+1,+1),Min_PWM_Out,Max_PWM_Out); //REAR_L  后左电机
    motor[3] = Math_Constrain(PIDMIX(+1,-1,-1),Min_PWM_Out,Max_PWM_Out); //FRONT_L 前左电机
	//xiang：这段代码是我改的，目的是在锁定油门后，依然可以在上位机上看到pwm输出
	if(THROTTLE_LOCKed)
	{
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		PWM_Output_CH1 =motor[0];
		PWM_Output_CH2 =motor[1];
		PWM_Output_CH3 =motor[2];
		PWM_Output_CH4 =motor[3];
		return;
	}else{
		Set_PWMOuput_CH1(motor[0]);
		Set_PWMOuput_CH2(motor[1]);
		Set_PWMOuput_CH3(motor[2]);
		Set_PWMOuput_CH4(motor[3]);
	}
    break;
  case Y4  :
    motor[0] = Math_Constrain(PIDMIX(+0,+1,-1),Min_PWM_Out,Max_PWM_Out);   //REAR_1 CW
    motor[1] = Math_Constrain(PIDMIX(-1,-1, 0),Min_PWM_Out,Max_PWM_Out); //FRONT_R CCW
    motor[2] = Math_Constrain(PIDMIX(+0,+1,+1),Min_PWM_Out,Max_PWM_Out);   //REAR_2 CCW
    motor[3] = Math_Constrain(PIDMIX(+1,-1, 0),Min_PWM_Out,Max_PWM_Out); //FRONT_L CW
	//xiang：这段代码是我改的，目的是在锁定油门后，依然可以在上位机上看到pwm输出
	if(THROTTLE_LOCKed)
	{
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		PWM_Output_CH1 =motor[0];
		PWM_Output_CH2 =motor[1];
		PWM_Output_CH3 =motor[2];
		PWM_Output_CH4 =motor[3];
		return;
	}else{
		Set_PWMOuput_CH1(motor[0]);
		Set_PWMOuput_CH2(motor[1]);
		Set_PWMOuput_CH3(motor[2]);
		Set_PWMOuput_CH4(motor[3]);
	}
    break;
  case Y6  :
    motor[0] = Math_Constrain(PIDMIX(+0,+4/3,+1),Min_PWM_Out,Max_PWM_Out); //REAR
    motor[1] = Math_Constrain(PIDMIX(-1,-2/3,-1),Min_PWM_Out,Max_PWM_Out); //RIGHT
    motor[2] = Math_Constrain(PIDMIX(+1,-2/3,-1),Min_PWM_Out,Max_PWM_Out); //LEFT
    motor[3] = Math_Constrain(PIDMIX(+0,+4/3,-1),Min_PWM_Out,Max_PWM_Out); //UNDER_REAR
    motor[4] = Math_Constrain(PIDMIX(-1,-2/3,+1),Min_PWM_Out,Max_PWM_Out); //UNDER_RIGHT
    motor[5] = Math_Constrain(PIDMIX(+1,-2/3,+1),Min_PWM_Out,Max_PWM_Out); //UNDER_LEFT  
	//xiang：这段代码是我改的，目的是在锁定油门后，依然可以在上位机上看到pwm输出
	if(THROTTLE_LOCKed)
	{
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		PWM_Output_CH1 =motor[0];
		PWM_Output_CH2 =motor[1];
		PWM_Output_CH3 =motor[2];
		PWM_Output_CH4 =motor[3];
		PWM_Output_CH5 =motor[4];
		PWM_Output_CH6 =motor[5];
		return;
	}else{
		Set_PWMOuput_CH1(motor[0]);
		Set_PWMOuput_CH2(motor[1]);
		Set_PWMOuput_CH3(motor[2]);
		Set_PWMOuput_CH4(motor[3]);
		Set_PWMOuput_CH5(motor[4]);
		Set_PWMOuput_CH6(motor[5]); 
	}
    break;
  case HEX6	:
    motor[0] = Math_Constrain(PIDMIX(-7/8,+1/2,+1),Min_PWM_Out,Max_PWM_Out); //REAR_R
    motor[1] = Math_Constrain(PIDMIX(-7/8,-1/2,-1),Min_PWM_Out,Max_PWM_Out); //FRONT_R
    motor[2] = Math_Constrain(PIDMIX(+7/8,+1/2,+1),Min_PWM_Out,Max_PWM_Out); //REAR_L
    motor[3] = Math_Constrain(PIDMIX(+7/8,-1/2,-1),Min_PWM_Out,Max_PWM_Out); //FRONT_L
    motor[4] = Math_Constrain(PIDMIX(+0  ,-1  ,+1),Min_PWM_Out,Max_PWM_Out); //FRONT
    motor[5] = Math_Constrain(PIDMIX(+0  ,+1  ,-1),Min_PWM_Out,Max_PWM_Out); //REAR
	//xiang：这段代码是我改的，目的是在锁定油门后，依然可以在上位机上看到pwm输出
	if(THROTTLE_LOCKed)
	{
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		PWM_Output_CH1 =motor[0];
		PWM_Output_CH2 =motor[1];
		PWM_Output_CH3 =motor[2];
		PWM_Output_CH4 =motor[3];
		PWM_Output_CH5 =motor[4];
		PWM_Output_CH6 =motor[5];
		return;
	}else{
		Set_PWMOuput_CH1(motor[0]);
		Set_PWMOuput_CH2(motor[1]);
		Set_PWMOuput_CH3(motor[2]);
		Set_PWMOuput_CH4(motor[3]);
		Set_PWMOuput_CH5(motor[4]);
		Set_PWMOuput_CH6(motor[5]); 
	}
    break;
  case HEX6X :
    motor[0] = Math_Constrain(PIDMIX(-1/2,+7/8,+1),Min_PWM_Out,Max_PWM_Out); //REAR_R
    motor[1] = Math_Constrain(PIDMIX(-1/2,-7/8,+1),Min_PWM_Out,Max_PWM_Out); //FRONT_R
    motor[2] = Math_Constrain(PIDMIX(+1/2,+7/8,-1),Min_PWM_Out,Max_PWM_Out); //REAR_L
    motor[3] = Math_Constrain(PIDMIX(+1/2,-7/8,-1),Min_PWM_Out,Max_PWM_Out); //FRONT_L
    motor[4] = Math_Constrain(PIDMIX(-1  ,+0  ,-1),Min_PWM_Out,Max_PWM_Out); //RIGHT
    motor[5] = Math_Constrain(PIDMIX(+1  ,+0  ,+1),Min_PWM_Out,Max_PWM_Out); //LEFT
	//xiang：这段代码是我改的，目的是在锁定油门后，依然可以在上位机上看到pwm输出
	if(THROTTLE_LOCKed)
	{
		PWM_Output_Set_default();//xiang：这行一定要有，因为锁油门的初衷就是输出为最低输出
		PWM_Output_CH1 =motor[0];
		PWM_Output_CH2 =motor[1];
		PWM_Output_CH3 =motor[2];
		PWM_Output_CH4 =motor[3];
		PWM_Output_CH5 =motor[4];
		PWM_Output_CH6 =motor[5];
		return;
	}else{
		Set_PWMOuput_CH1(motor[0]);
		Set_PWMOuput_CH2(motor[1]);
		Set_PWMOuput_CH3(motor[2]);
		Set_PWMOuput_CH4(motor[3]);
		Set_PWMOuput_CH5(motor[4]);
		Set_PWMOuput_CH6(motor[5]); 
	}
  	break;
  }

}

//------------------End of File----------------------------
