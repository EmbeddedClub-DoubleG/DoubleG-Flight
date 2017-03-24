#include "Target.h"
#include "IMU.h"
#include "Quadrotor.h"

int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
float  Target_Roll,Target_Pitch,Target_Yaw,Tartget_hight = 0.0;//Target_Roll,Target_Pitch单位为 x/10=度

#if Yingzhang_GCS
//上位机控制四轴目标角度相关变量
int16_t GCSControl_CH1 = 0;//中立值为0
int16_t GCSControl_CH2 = 0;
int16_t GCSControl_CH3 = 0;
int16_t GCSControl_CH4 = 0;
#endif

/*
PWM输入定义：
	PWM_Input_CH1 目标横滚角度输入  1500us为0度
	PWM_Input_CH2 目标俯仰角度输入  1500us为0度
	PWM_Input_CH3 油门量输入
	PWM_Input_CH4 航向输入
	PWM_Input_CH5、PWM_Input_CH6 飞行模式控制
	油门量CH3大于1500us时，进入电调行程设置模式
*/
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


/**************************实现函数********************************************
	*函数原型:	void Get_Tartget_RPY(void)
	*功　　能:  控制信号的转换。读取遥控器和上位机信号，转换为相应的roll、pitch、yaw、Throttle
*******************************************************************************/
void Get_Tartget_RPY(void)
{
	float ftemp=0;
	Target_Roll = (int16_t)(PWM_Input_CH1 - PWM_Input_Offset);//将通道1 输入做为Roll的目标角度
	Target_Pitch = (int16_t)(PWM_Input_CH2 - PWM_Input_Offset);//将通道2 输入做为Pitch的目标角度
	//将通道4 输入做为目标航向输入	[<1400us || >1600]有效
	if((PWM_Input_CH4 > (int16_t)(PWM_Input_Offset+100))||(PWM_Input_CH4 < (int16_t)(PWM_Input_Offset-100)))
	{
		ftemp = (float)(PWM_Input_CH4 - PWM_Input_Offset); //[-500,+500]
		Target_Yaw += (ftemp / 500.0f)*0.2f; //每次改变的角度范围为-0.2~0.2 小溪++等于大河
		if(Target_Yaw >180.0f)
			Target_Yaw = Target_Yaw-360.0f;	//转[-180.0,+180.0]
		else if(Target_Yaw <-180.0f)
			Target_Yaw = 360.0f + Target_Yaw;
	}
	#if Yingzhang_GCS
	//ch1增加往右，ch2增加往前，ch4增加往右
	if(GCSControl_Leftward)		Target_Roll-=50;
	if(GCSControl_Rightward)	Target_Roll+=50;
	if(GCSControl_Forward)		Target_Pitch+=50;
	if(GCSControl_Backward)		Target_Pitch-=50;
	// if(GCSControl_LeftRotate)	Target_Yaw-=5;
	// if(GCSControl_RightRotate)	Target_Yaw+=5;
	#endif
}

/**************************实现函数********************************************
	*函数原型:	void Lock_Target_Yaw(void)
	*功　　能:  目标航向控制。当油门大于1100us时，认为用户希望起飞。那么此时的航向做为目标航向
*******************************************************************************/
void Lock_Target_Yaw(void)
{
	static uint8_t yaw_lock = 0;
	if(PWM_Input_CH3 > (int16_t)1100)
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
