/* Ultrasonic.c file

占用资源：
1. 超声波控制引脚
	PA12  -->  Trig  触发发送超声波 高电平有效
	PA11  -->  Echo  超声波信号有效  高电平有效
2. 引脚变化外部中断线  EXTI15_10_IRQn  
3. 读取系统时间API
4. 得到当前环境的温度，以计算声音的速度	 如果超声波模块有温度补偿，按说明书上的声速操作

功能：
读取超声波的测距高度。 相应的API 见	Ultrasonic.h
------------------------------------
 */

#include "Ultrasonic.h"
#include "delay.h"
#include "MS5611.h"
#include "IMU.h"//update20161227:由于增加了超声波消除俯仰和横滚影响，所以使用了IMU_Roll和IMU_Pitch
#include "AltitudeFilter.h"
#include <math.h>

//------------超声波驱动状态机--------------
#define Ultra_ST_Started   0x01	  //启动测距
#define Ultra_ST_RX1       0x02	  //收到回波1
#define Ultra_ST_RX2       0x03	  //收到回波2
#define Ultra_ST_Idle      0x04	  //休息一下
#define Ultra_Restart      0x1f	  //重新再来一次测距
#define Ultra_ST_Error     0xf2	  //错误

#define MOVAVG_SIZE  10	   //保存最近的10个数据 进行平均滤波
#define Max_Range	3.5f
#define Time_outed        (uint32_t)(Max_Range*2)*1000000/340  // 超时时间 单位us
// low pass filter:
// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 5  Hz -> _filter = 31.8310e-3
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
const float  Ultra_Lowpass = 31.8310e-3f;  //低通滤波系数 20hz//update201612271359

volatile uint8_t Ultra_Stauts = Ultra_ST_Idle; //当前状态
volatile uint32_t Ultra_High_time = 0,
				   Ultra_Start_time = 0,	
				   Ultra_Low_time = 0;

static float Dist_buffer[MOVAVG_SIZE];
static uint8_t Dis_index = 0;

float Ultra_dt = 0.0; //两次测量的时间差作为Ultra_dt 单位us
float  Ultra_Distance = 0 ;//xiang：全局变量，当前检测到的高度值，单位是米；超声波这一块所有的距离单位都是米;本来是初始化为0，为了方便观察，我设置为0.01
uint8_t Ultra_ALT_Updated = 0; //高度更新完成标志。
int16_t Ultra_Healthy = 0;//当数据在量程外时--，在量程内时++；范围为0~20；当值在15~20内时认为超声波健康
//添加一个新的值到 队列 进行滤波
void Ultrasonic_NewDis(float val)
{
    static uint32_t last_time = 0;
    uint32_t temp;
    if (val > (float)Max_Range)
		return;
    temp = micros();
    if (temp <= last_time)
		Ultra_dt = ((float)(temp + (0xffffffff - last_time))) / 1000000.0f;
    else
		Ultra_dt = ((float)(temp - last_time)) / 1000000.0f;
    last_time = temp;
    Dist_buffer[Dis_index] = val;
    Dis_index = ((Dis_index + 1) % MOVAVG_SIZE);
}

//读取队列 的平均值
float Ultrasonic_getAvg(float * buff, int size) {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return (sum / (float)size);
}

// 配置中断线  EXTI15_10_IRQn 优先级别 并使能
void Ultrasonic_NVIC_Config(void){
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//初始化超声波对应的引脚和中断。
void Ultrasonic_initial(void){
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	uint8_t i;
	/* config the extiline(PA0) clock and AFIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	Ultrasonic_NVIC_Config(); //中断设置

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;       
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* EXTI line(PA11) mode config */
  	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource11);

  	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_12);	//先不要发超声波

	for(i=0;i<MOVAVG_SIZE;i++){
				Dist_buffer[i]=0; //清队列，准备行动！
				}
}

//启动 发送超声波信号。
void Ultrasonic_Start(void){
   GPIO_SetBits(GPIOA, GPIO_Pin_12); //高电平 触发超声波
   delay_us(12);  //延时，应大于 10us 
   GPIO_ResetBits(GPIOA, GPIO_Pin_12); //够了，不要再发了。
   Ultra_Stauts = Ultra_ST_Started;	 //改状态，
   Ultra_Start_time = micros();	//取时间
}

//超声波测距的线程， 此程序需要定时调用。以更新距离
void Ultrasonic_Routine(void){
	uint32_t temp;
	static uint32_t HaltTime = 0;
	static int16_t Ultra_valid = 0;
	float Distance;
	//计算在当前温度下 对应的空气中声音的传播速度
	/*
	音速与介质的密度和弹性性质有关，因此也随介质的温度、
	压强等状态参量而改变。气体中音速每秒约数百米，
	随温度升高而增大，0℃时空气中音速为331．4米／秒，
	15℃时为340米／秒，温度每升高1℃，音速约增加0．6米／秒。
	*/
	float Sound_Speed;
	switch(Ultra_Stauts){ //查询当前状态
   	case Ultra_ST_Started: 	 //已经启动测量了
			temp = micros();
			if((temp - Ultra_Start_time)>Time_outed){
			Ultra_Stauts = Ultra_ST_Error;	 //超时了，错误
			}
			break;
	case Ultra_ST_RX1: 	//已经收到了高电平
			temp = micros();
			if((temp - Ultra_Start_time)>Time_outed){
			Ultra_Stauts = Ultra_ST_Error; //超时了，错误	
			}
			break;
	case Ultra_ST_RX2: //已经收到低电平。可以计算距离了
			Sound_Speed = (332.0f+ (MS5611_Temperature/100.0f)*0.607f);//计算声速
			Distance = (float)(Ultra_Low_time - Ultra_High_time);  //时间   单位 us
			Distance = (Distance/2000000.0f); //计算 声音走一半，需要的时间，单位 S
			Distance = (Distance) * Sound_Speed; //距离			

			if ((Distance > (float)Max_Range) || (Distance < 0.01f)) //update20161227:把判断条件从<0改为<0.01
			{
			    Ultra_Stauts = Ultra_ST_Error; //超时了，错误
			    break; //超过量程了，这次测量无效。
			}
			Distance *= (float)cos(fabs(IMU_Roll * M_PI / 180));//fabs是取绝对值//update20161227:消除角度影响；这个代码需不需要加有争议，因为cos计算量大，但是收益不高
			Distance *= (float)cos(fabs(IMU_Pitch * M_PI / 180));

			Ultra_Distance = Ultrasonic_getAvg(Dist_buffer,MOVAVG_SIZE);
			Ultra_Distance = Ultra_Distance + //低通滤波   20hz
					 (Ultra_dt / (Ultra_dt + Ultra_Lowpass)) * (Distance - Ultra_Distance);
			Ultrasonic_NewDis(Ultra_Distance);
			if(++Ultra_Healthy>20)
			    Ultra_Healthy = 20;
			if(++Ultra_valid>MOVAVG_SIZE){ //连续MOVAVG_SIZE次超声波高度有效，那么应该用它来修正气压高度的漂移
				MS561101BA_SetAlt(Ultra_Distance);  //超声波 高度有效。标定气压高度。xiang：这里不仅标定了气压高度，还把超声波的数据存到了气压计的buffer里
				Ultra_valid = MOVAVG_SIZE;
			}
			Ultra_ALT_Updated = 1;
			Get_Filter_Altitude();
			Ultra_Stauts = Ultra_ST_Idle;
			break;
	case Ultra_ST_Error:
			if(--Ultra_Healthy<0)
			    Ultra_Healthy = 0;
			Ultra_valid = 0;  //超声波高度有效计数 清零
			Ultra_Stauts = Ultra_ST_Idle; //进入休息状态
			break;
	case Ultra_ST_Idle:		 //休息时间。我们不要采集那么快。
			if(HaltTime == 0){
				HaltTime =	micros();
			}else
			{
				temp =	micros();
				if((temp - HaltTime)>Time_outed){
					Ultra_Stauts = Ultra_Restart; //休息够了，重新启动一次采集
					HaltTime = 0;
				}
			}
		 break;
	case Ultra_Restart:	   //重启一次测距
		 Ultrasonic_Start();  //发送超声波信号
		 Ultra_Stauts = Ultra_ST_Started; // 改变状态，
		 break;
	default: 
		   Ultrasonic_Start();
		   Ultra_Stauts = Ultra_ST_Started;
		   break;
	}

}

//中断线15-10 的中断程序
void EXTI15_10_IRQHandler(void)
{
	if( GPIOA->IDR & 0x0800){	//PA11   高电平否？
	 Ultra_High_time = micros();
	 Ultra_Stauts = Ultra_ST_RX1; //我们收到了高电平 状态机变化	
	}else{		//低电平
	 Ultra_Low_time = micros();
	 Ultra_Stauts = Ultra_ST_RX2;	//采样结束  
	}
   EXTI->PR=1<<11;  //清除中断标志位 
}

//------------------End of File----------------------------
