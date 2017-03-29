/*
PWM输入定义：
	PWM_Input_CH1 目标横滚角度输入  1500us为0度
	PWM_Input_CH2 目标俯仰角度输入  1500us为0度
	PWM_Input_CH3 油门量输入
	PWM_Input_CH4 航向输入
	PWM_Input_CH5、PWM_Input_CH6 飞行模式控制
	油门量CH3大于1500us时，进入电调行程设置模式
启动时如果遥控器没有打开，那么ch1-ch6的输入都为0，ch1-ch4的输出都为800，ch5-ch8的输出都为1000；
之后打开遥控器，ch5-ch8的输出都为1000，其他的都正常；
之后关闭遥控器，ch1-ch6的输入保持与关闭遥控器瞬间数值+-1的变化，并没有清为0，ch1-ch4的输出正常。
*/
#include "Target.h"
#include "IMU.h"
#include "Quadrotor.h"
#include "PWM_Input.h"

int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
float Target_Roll,Target_Pitch,Target_Yaw;//Target_Roll,Target_Pitch单位为 x/10=度
volatile float Quad_THR = MINTHROTTLE;
// volatile uint8_t IsAutoHight = 0;//是否开启定高

#if Yingzhang_GCS
//上位机控制四轴目标角度相关变量
int16_t GCSControl_CH1 = 0;//中立值为0
int16_t GCSControl_CH2 = 0;
float GCSControl_CH3 = 0;//起始值为0
float GCSControl_CH3_Accumulate = 0;
int16_t GCSControl_CH4 = 0;
#endif

/**************************实现函数********************************************
*函数原型:	void PWM_Save_Offset(void)
*功　　能:	上位机标记中立值
*******************************************************************************/
void PWM_Save_Offset(void)
{
   PWM_Offset_Roll = PWM_Input_CH1 + PWM_Offset_Roll - 1500;
   PWM_Offset_Pitch = PWM_Input_CH2 + PWM_Offset_Pitch - 1500; 
   PWM_Offset_Yaw = PWM_Input_CH4 + PWM_Offset_Yaw - 1500;
   AT45DB_Write_config();
   //LED_Set_Blink(Blue,50,50,4);
}
void Get_Throttle(void)
{
	if (PWM_Input_CH3 > (int16_t)PWM_Input_MIN && PWM_Input_CH3 < (int16_t)PWM_Input_MAX)
	{
		Quad_THR = PWM_Input_CH3;
	}
	else
	{
	    Quad_THR = MINTHROTTLE;
	}
	#if Yingzhang_GCS
	GCSControl_CH3 += GCSControl_CH3_Accumulate;
	if (GCSControl_CH3 + Quad_THR > MAXTHROTTLE)
	    GCSControl_CH3 = MAXTHROTTLE - Quad_THR;
	if (GCSControl_CH3 + Quad_THR < MINTHROTTLE)
	    GCSControl_CH3 = MINTHROTTLE - Quad_THR;
	Quad_THR += GCSControl_CH3;
	#endif
}

/**************************实现函数********************************************
	*函数原型:	void Get_Tartget_RPY(void)
	*功　　能:  控制信号的转换。读取遥控器和上位机信号，转换为相应的roll、pitch、yaw、Throttle
*******************************************************************************/
void Get_Tartget_RPY(void)
{
//ch1增加往右，ch2增加往前，ch4增加往右
	float ftemp=0;
	if (PWM_Input_CH3 > (int16_t)PWM_Input_MIN && PWM_Input_CH3 < (int16_t)PWM_Input_MAX)
	{
		Target_Roll = (int16_t)(PWM_Input_CH1 - PWM_Input_Offset);//将通道1 输入做为Roll的目标角度
		Target_Pitch = (int16_t)(PWM_Input_CH2 - PWM_Input_Offset);//将通道2 输入做为Pitch的目标角度
		if ((PWM_Input_CH4 > (int16_t)(PWM_Input_Offset + 100)) || (PWM_Input_CH4 < (int16_t)(PWM_Input_Offset - 100)))
		{
		    ftemp = (float)(PWM_Input_CH4 - PWM_Input_Offset); //[-500,+500]
		    Target_Yaw += (ftemp / 500.0f) * 0.2f;	     //每次改变的角度范围为-0.2~0.2 小溪++等于大河
			if (Target_Yaw > 180.0f)
				Target_Yaw = Target_Yaw - 360.0f; //转[-180.0,+180.0]
			else if (Target_Yaw < -180.0f)
				Target_Yaw = 360.0f + Target_Yaw;
		}
	}
	else
	{
	    Target_Roll = 0;
	    Target_Pitch = 0;
	}
	#if Yingzhang_GCS
	Target_Roll += GCSControl_CH1;
	Target_Pitch += GCSControl_CH2;
	Target_Yaw += (GCSControl_CH4 / 500.0f) * 0.2f; //每次改变的角度范围为-0.2~0.2 小溪++等于大河
	if (Target_Yaw > 180.0f)
	    Target_Yaw = Target_Yaw - 360.0f; //转[-180.0,+180.0]
	else if (Target_Yaw < -180.0f)
	    Target_Yaw = 360.0f + Target_Yaw;
	#endif
}

/**************************实现函数********************************************
	*函数原型:	void Lock_Target_Yaw(void)
	*功　　能:  目标航向控制。当油门大于1100us时，认为用户希望起飞。那么此时的航向做为目标航向
*******************************************************************************/
void Lock_Target_Yaw(void)
{
	static uint8_t yaw_lock = 0;
	if(Quad_THR > (int16_t)1100)
	{
		if(yaw_lock != 1){
			yaw_lock = 1;
			Target_Yaw = IMU_Yaw; //将当前的航向做为目标航向
		}
	}
	else{
		yaw_lock = 0;
	}
}

/**************************实现函数********************************************
	*函数原型:	unsigned char Read_Mode(void)
	*功　　能:  读取PWM5\PWM6输入值，确定飞控的工作模式
	输出参数：飞行模式标识
*******************************************************************************/
unsigned char Read_Mode(void)
{
	if(PWM_Input_CH6 < (int16_t)(PWM_Input_Offset-100))//CH6小于 1400us
	{
		if(PWM_Input_CH5 < (int16_t)(PWM_Input_Offset-100))//CH5小于 1400us
		//左下右上：
			return Quad_Level_Lock;//平衡模式
		else if(PWM_Input_CH5 >(int16_t)(PWM_Input_Offset+100))//CH5大于 1600us
		//左上右上
			return Quad_Auto_High;//定高模式
	}
	else if(PWM_Input_CH6 >(int16_t)(PWM_Input_Offset+100))//CH6大于 1600us
	{
		if(PWM_Input_CH5 < (int16_t)(PWM_Input_Offset-100))//CH5小于 1400us
		//左下右下：
			return 	Quad_Hold_Position ;//定点模式
		else if(PWM_Input_CH5 >(int16_t)(PWM_Input_Offset+100))//CH5大于 1600us
		//左上右下：
			return Quad_Manual;//手动模式
	}
	return Quad_Manual;
}
