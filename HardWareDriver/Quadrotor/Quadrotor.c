#include "Quadrotor.h"
#include "Fmath.h"
// #include "math.h"
#include "GPS.h"
#include "GCS_Protocol.h"
#include "Quad_PID.h"
#include "Target.h"

volatile uint8_t  Fly_Mode = default_quad; //飞行器的架构，默认十字四轴，见fly_config.h
volatile int16_t  Yaw_DIRECT = 1;//航向是否取反
int16_t THROTTLE = MINTHROTTLE, PID_ROLL = 0, PID_PITCH = 0, PID_YAW = 0;
volatile float PID_dt = 0;
volatile float Quad_Pitch,Quad_Roll,Quad_Yaw,Quad_THR=MINTHROTTLE;//update20161226:添加了Quad_Pitch,Quad_Roll,Quad_Yaw这几个变量，虽然没用到
volatile uint8_t Quadrotor_Mode = Quad_Manual; 	//当前的飞行模式，手动，平衡，定点等

//定高相关变量
//光流定点相关全局变量
volatile float Increas_Xspeed_Accumulat=0;//增量式pid的输出的累加
volatile float Increas_Yspeed_Accumulat=0;//增量式pid的输出的累加
float RollAdd,PitchAdd;


/*---------------------内部函数声明------------------------*/

uint8_t Change_Mode(unsigned char);
void Led_Quad_Mode(void);

void Mode_Manual(void);
void Mode_Level_Lock(void);
void Mode_Hold_Position(void);
void Mode_ESC_Cal(void);
void Mode_Assignment(void);
void Mode_Landing(void);
void Mode_Take_Of(void);
void Mode_Auto_High(void);

/**************************实现函数********************************************
*函数原型:		void Quadrotor_Motor_Update(void)
*功　　能:	    电机转速更新。四轴的关键控制部分.
输出的结果都在	Set_PWMOuput_CH1-6 直接输出PWM
*******************************************************************************/
void Quadrotor_Motor_Update(void)
{

	static unsigned char mode;//读取遥控器输入的飞行模式，但是不一定作为真的飞行模式，得先做一些判断
	static uint32_t last_time = 0;
	uint32_t now_time;
	//计算控制环更新时间间隔
	if(last_time == 0){
		last_time = micros();
		return ;
	}
	now_time = micros(); //读取系统时间
	if(now_time <= last_time){
		PID_dt = ((float)(now_time + (0xffffffff - last_time))) / 1000000.0f;
	}else{
		PID_dt = ((float)(now_time - last_time)) / 1000000.0f;
	}
	last_time=now_time;
	
	mode=Read_Mode();//读取工作模式。通过PWM5、PWM6输入值判断
	Change_Mode(mode);//切换飞行模式
	Lock_Target_Yaw();//目标航向控制。当油门大于1100us时，认为用户希望起飞。那么此时的航向做为目标航向	
	Led_Quad_Mode();//LED指示飞行模式
	
//--------------------相应的飞行模块PWM输出运算-------------------------------------
	switch(Quadrotor_Mode)//检测当前状态
	{
		case Quad_Manual:			Mode_Manual();//手动模式
			break;		
		case Quad_Level_Lock:		Mode_Level_Lock();//平衡飞行状态
			break;		
		case Quad_Hold_Position:	Mode_Hold_Position();//定点飞行模式
			break;	
		case Quad_ESC_Cal:			Mode_ESC_Cal();//ESC 电子调速器校准  必须在手动模式下进行
			break;		
		case Quad_Assignment:		Mode_Assignment();//任务模式
			break;		
		case Quad_Landing:			Mode_Landing();//降落模式
			break;		
		case Quad_Take_Of:			Mode_Take_Of();//起飞模式
			break;
		case Quad_Auto_High:		Mode_Auto_High();//定高模式
			break;		
		default :
			Quadrotor_Mode = Quad_Manual;
			PWM_Output_Set_default();
			break;
	}
}

/**************************实现函数********************************************
*函数原型:		void Initial_Quadrotor_Math(void)
*功　　能:	    初始化四轴的数据参数。
读取PID参数，以便在四轴控制中进行运算
*******************************************************************************/
void Initial_Quadrotor_Math(void){

	AT45DB_Read_config(); //读取配置  {调用AT45DB.c的子程序} 

	//读取飞行器架构
	if((Config.fly_mode > 0x00)&&(Config.fly_mode < 0x08)){
		Fly_Mode = Config.fly_mode;
		}else Fly_Mode = default_quad; //错误的配置信息，只好装载默认值

	//读取航向控制方向
	if(Config.yaw_dir == 0)	  //航向反向？
			Yaw_DIRECT = (int16_t)1;
		else if(Config.yaw_dir == 1)
			Yaw_DIRECT = (int16_t)-1;
			else Yaw_DIRECT = (int16_t)YAW_DIRECTION;

	//初始化PID控制器	将P I D三个参数注入到控制器中。
	pidInit(&Stabilize_Roll,Config.Angle_Roll_P,
							Config.Angle_Roll_I,
							Config.Angle_Roll_D);
	pidInit(&Stabilize_Pitch,Config.Angle_Pitch_P,
							Config.Angle_Pitch_I,
							Config.Angle_Pitch_D);
	pidInit(&RollRate,      Config.Rate_Roll_P,
							Config.Rate_Roll_I,
							Config.Rate_Roll_D);
	pidInit(&PitchRate,     Config.Rate_Pitch_P,
							Config.Rate_Pitch_I,
							Config.Rate_Pitch_D);
	pidInit(&Stabilize_Yaw, Config.Angle_Yaw_P,
							Config.Angle_Yaw_I,
							Config.Angle_Yaw_D);
	pidInit(&YawRate,       Config.Rate_Yaw_P,
							Config.Rate_Yaw_I,
							Config.Rate_Yaw_D);
	pidInit(&AutoHigh_THR,  Config.High_P,
							Config.High_I,
							Config.High_D);
	pidInit(&Climb,     	Config.Climb_High_P,
							Config.Climb_High_I,
							Config.Climb_High_D);
	pidInit(&Position_Hold, Config.Position_Hold_P,
							Config.Position_Hold_I,
							Config.Position_Hold_D);
	pidInit(&Position_Speed,Config.Position_Speed_P,
							Config.Position_Speed_I,
							Config.Position_Speed_D);

	pidSetLowPassEnable(&RollRate);	
	pidSetLowPassEnable(&PitchRate);
	pidSetLowPassEnable(&YawRate);
	pidSetLowPassEnable(&Climb);
	pidSetIntegralLimit(&AutoHigh_THR , 30.0f);//xiang:本来是10
	pidSetIntegralLimit(&Climb , 200.0f);//xiang:本来是100
	pidSetIntegralLimit(&Z_Speed , 100.0f);//update20170110

	if((Config.pwm_in_offset1>1800)||(Config.pwm_in_offset1<1200)){
		PWM_Offset_Roll = 1500;
		PWM_Offset_Pitch = 1500;
		PWM_Offset_Yaw = 1500;
		}else{
		PWM_Offset_Roll = Config.pwm_in_offset1;
		PWM_Offset_Pitch = Config.pwm_in_offset2;
		PWM_Offset_Yaw = Config.pwm_in_offset4;
	}
	Tartget_hight = Config.target_hight; //装载目标高度值
}


/**************************实现函数********************************************
*函数原型:		void PWM_Output_ESC_Calibration(void)
*功　　能:	    ESC电子调速器 校准
所有的通道都输出  PWM输出3 的值，用户此时可以通过油门通道设置电调的行程
*******************************************************************************/
void PWM_Output_ESC_Calibration(void) {

	Set_PWMOuput_CH1(PWM_Input_CH3); //全部输出CH3油门量。
	Set_PWMOuput_CH2(PWM_Input_CH3);
	Set_PWMOuput_CH3(PWM_Input_CH3);
	Set_PWMOuput_CH4(PWM_Input_CH3);
	PWM_Output_CH1 = PWM_Input_CH3;
	PWM_Output_CH2 = PWM_Input_CH3;
	PWM_Output_CH3 = PWM_Input_CH3;
	PWM_Output_CH4 = PWM_Input_CH3;

	if((Fly_Mode == Y6)||(Fly_Mode == HEX6X)||(Fly_Mode == HEX6)){
	Set_PWMOuput_CH5(PWM_Input_CH3);
	Set_PWMOuput_CH6(PWM_Input_CH3);
	PWM_Output_CH5 = PWM_Input_CH3;
	PWM_Output_CH6 = PWM_Input_CH3;
	}
}

/**************************实现函数********************************************
	*函数原型:	uint8_t Change_Mode(unsigned char)
	*功　　能:  切换飞行模式
	输入参数：待决定的飞行模式标识
	输出参数：已决定的飞行模式标识
*******************************************************************************/
uint8_t Change_Mode(unsigned char mode)
{
	if((Quadrotor_Mode != mode)&&(Quadrotor_Mode != Quad_ESC_Cal))
	{
		Quadrotor_Mode = mode;
		//LED_Set_Blink(Blue,100,200,4);
		pidReset(&Stabilize_Roll); //清PID 的积分
		pidReset(&Stabilize_Pitch);
		pidReset(&Stabilize_Yaw);
		// pidReset(&AutoHigh_THR);//定高
		// pidReset(&Climb);//爬升率
		Position_Hold_Reset();
	}
	return Quadrotor_Mode;
}
/**************************实现函数********************************************
	*函数原型:	void Led_Quad_Mode(void)
	*功　　能:  LED指示飞行状态
*******************************************************************************/
void Led_Quad_Mode(void)
{
	static int16_t led_mode_count=0;
	led_mode_count++;
	if(led_mode_count >= PWM_Hz)//1S 进入一次，用红色的LED提示当前飞行模式
	{
		led_mode_count = 0;
		switch(Quadrotor_Mode)//检测当前状态
		{
			case Quad_ESC_Cal: LED_Set_Blink(Red,50,50,10); 	
					break;	//电调校准模式，红色的LED快速闪烁
			
			case Quad_Manual: LED_Set_Blink(Red,100,100,1);
					break;	//手动模式  红色的LED 1S 闪一次
			case Quad_Level_Lock: LED_Set_Blink(Red,100,100,2);
					break;	//平衡模式  红色的LED 1S 闪两次
			case Quad_Hold_Position: LED_Set_Blink(Red,100,100,3);
					break;  //定点模式  红色的LED 1S 闪三次
			case Quad_Assignment: LED_Set_Blink(Red,100,100,4);
					break;  //任务模式  红色的LED 1S 闪四次		
			case Quad_Landing: LED_Set_Blink(Red,100,100,5);
					break;  //降落模式  红色的LED 1S 闪5次		
			case Quad_Take_Of: LED_Set_Blink(Red,100,100,6);
					break;  //起飞模式  红色的LED 1S 闪6次
		}
	}
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Manual(void)
	*功　　能:  手动模式
*******************************************************************************/
void Mode_Manual(void)
{
	//手动时 控制信号的混合
	THROTTLE = (int16_t)(PWM_Input_CH3); //取油门量
	PID_PITCH = (int16_t)(PWM_Input_CH2 - PWM_Input_Offset); //取pitch
	PID_ROLL = (int16_t)(PWM_Input_CH1 - PWM_Input_Offset); //取Roll
	PID_YAW =  PWM_Input_CH4 - PWM_Input_Offset; //取Yaw
	//手动模式 加入陀螺仪增稳功能
	Roll_Pitch_Yaw_RatePID(0,0,0);
	PID_YAW   += YawRate.PID_out;
	PID_PITCH += PitchRate.PID_out;
	PID_ROLL += RollRate.PID_out;
	PWM_Write_Motors(); //写输出到PWM信号。
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Level_Lock(void)
	*功　　能:  平衡模式
*******************************************************************************/
void Mode_Level_Lock(void)
{
	Get_Tartget_RPY();//遥控信号的转换
	Quad_THR = (int16_t)(PWM_Input_CH3); //将通道3 输入做为油门量
	//--------计算PWM输出---------
	Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
	
	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;
	//写输出到比较器  改写PWM输出脉宽值

	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Hold_Position(void)
	*功　　能:  定点模式
*******************************************************************************/
void Mode_Hold_Position(void)
{
//GPS定点
	// Get_Tartget_RPY();//遥控信号的转换
	// Quad_THR = (int16_t)(PWM_Input_CH3); //将通道3 输入做为油门量
	// //-------------定点飞行------------	
	// //xiang：定点飞行模式除了添加了这一段代码，其他的和平衡模式一样；而这一段代码主要和GPS有关。
	// if(Home_Ready)
	// {    //是否保存了家点。Home_Ready变量在GPS.c中定义
	// 	if(GPS_Update)
	// 	{	//GPS数据有更新。
	// 		GPS_Position_Hold();  //重新计算定点值。
	// 		GPS_Update = 0;			
	// 	}
	// 	 Target_Roll += GPS_ROLL; //如果没有安装GPS，程序不会运行到这里
	// 	 Target_Pitch += GPS_PITCH;
	// }
	// else
	// {
	// 	 Position_Hold_Reset();
	// }
	// //--------通过PID计算PWM输出-------
	// if(PWM_Input_CH3 > (int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10))
	// {
	// 	//遥控器油门大于10% 定高才会起作用，防止误动作引起电机转动
	// 	Quad_THR += Height_PID(Tartget_hight);
	// }
	// else
	// {
	// 	pidReset(&Climb);
	// 	pidReset(&AutoHigh_THR);
	// 	pidReset(&Z_Speed);		
	// }
	// Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
	// PID_PITCH = PitchRate.PID_out;
	// PID_ROLL = RollRate.PID_out;
	// PID_YAW = YawRate.PID_out;
	// THROTTLE = Quad_THR;
	// //写输出到比较器  改写PWM输出脉宽值

	// PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	
//营长:以下代码是我用光流模块写的悬停
	float Output = 0;
	float Position_SpeedTarget=0;
	float Position_MoveTarget=0;
	static float Position_X_MoveErr=0,Position_X_SpeedErr=0;
	static float Position_Y_MoveErr=0,Position_Y_SpeedErr=0;
	static uint32_t last_call_us = 0;
	float Interval_dt = 0;
  uint32_t now_time = micros();
  if(now_time - last_call_us > 100000 )
		{ //超过100ms没有调用这个程序了。
			Position_X_MoveErr=0;
			Position_X_SpeedErr=0;
			Position_Y_MoveErr=0;
			Position_Y_SpeedErr=0;
			last_call_us = 0;
			Xmove=0;
			Ymove=0;
    }

  Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
  last_call_us = now_time;
	//以下部分为定高(Z轴)
	Quad_THR = (int16_t)PWM_Input_CH3; //将通道3 输入做为油门量
	
	Get_Tartget_RPY();//遥控信号的转换
//	IMU_Roll += 2.0;

//	Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);

	if (PWM_Input_CH3 > (int16_t)(MINTHROTTLE + (MAXTHROTTLE - MINTHROTTLE) / 10))
	{ //遥控器油门大于10% 定高才会起作用，防止误动作引起电机转动
	    Quad_THR += Height_PID(MS5611_Altitude/100.0f);
	}
	else
	{
	    Height_PID_Reset();
	}

//	PID_PITCH = PitchRate.PID_out;
//	PID_ROLL = RollRate.PID_out;
//	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;
	
	//PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	
	//以下部分为定点(XY轴)
	pidSet(&Position_X_Hold ,1, 0, 0.00);
	pidSet(&Position_X_Speed,3, 0.0, 0.01);
	pidSet(&Position_Y_Hold, 1, 0, 0.00);
	pidSet(&Position_Y_Speed, 3, 0.0, 0.01);
	if (PWM_Input_CH3 > (int16_t)(MINTHROTTLE + (MAXTHROTTLE - MINTHROTTLE) / 10))
	{ //遥控器油门大于10% 定点才会起作用，防止误动作引起电机转动
		if((Target_Roll > (int16_t)-30)&&(Target_Roll < (int16_t)30))
		{
//			pidSetTarget_Measure(&Position_Hold,0,Xmove);
//			Position_MoveErr = Position_MoveErr
//			+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Position_Hold.merror , -3000.0f , 3000.0f) - Position_MoveErr);
//			Position_SpeedTarget = pidUpdate_err(&Position_Hold,Position_MoveErr,Interval_dt);
//			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-1000,+1000);
//			pidSetTarget_Measure(&Position_Speed,Position_SpeedTarget,X_Speed);
//			Position_SpeedErr = Position_SpeedErr + //低通滤波。 2Hz
//    		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Position_Speed.merror, -1000.0f , 1000.0f) - Position_SpeedErr);
//			Output = pidUpdate_err(&Position_Speed,Position_SpeedErr,Interval_dt)/100;
//			Output = Math_fConstrain(Output,-10,10);
//			Target_Roll = (- Output) * 10;
//			RollAdd = Output;  //记录每次角度的变化，传到上位机调试用
			
//			pidSetTarget_Measure(&Position_X_Hold,0,Xmove);
//			Position_X_MoveErr = Position_X_MoveErr
//			+ (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_X_Hold.merror , -1200.0f , 1200.0f) - Position_X_MoveErr);
//			Position_SpeedTarget = pidUpdate_err(&Position_X_Hold,Position_X_MoveErr,Interval_dt);
//			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-200,+200);
//			pidSetTarget_Measure(&Position_X_Speed,Position_SpeedTarget,X_Speed);
//			Position_X_SpeedErr = Position_X_SpeedErr
//			+ (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_X_Speed.merror , -500.0f , 500.0f) - Position_X_SpeedErr);
//			Output = pidUpdate_err(&Position_X_Speed,Position_X_SpeedErr,Interval_dt)/10000.0;
//			Output = Math_fConstrain(Output,-0.05,0.05);
//			Increas_Xspeed_Accumulat += Output;//增量式pid的输出的累加
//			Increas_Xspeed_Accumulat = Math_fConstrain(Increas_Xspeed_Accumulat,-2.5,+2.5);
//			Target_Roll = (- Increas_Xspeed_Accumulat) * 10;
//			RollAdd = Increas_Xspeed_Accumulat;  //记录每次角度的变化，传到上位机调试用
			
		  pidSetTarget_Measure(&Position_X_Hold,0,Xmove);
			Position_X_MoveErr = Position_X_MoveErr
			+ (Interval_dt / (15.9155e-3f + Interval_dt)) * (Math_fConstrain(Position_X_Hold.merror , -1000.0f , 1000.0f) - Position_X_MoveErr);
			Position_SpeedTarget = pidUpdate_err(&Position_X_Hold,Position_X_MoveErr,Interval_dt);
			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-200,+200);
			pidSetTarget_Measure(&Position_X_Speed,Position_SpeedTarget,X_Speed);
			Position_X_SpeedErr = Position_X_SpeedErr
			+ (Interval_dt / (15.9155e-3f + Interval_dt)) * (Math_fConstrain(Position_X_Speed.merror , -250.0f , 250.0f) - Position_X_SpeedErr);
			Output = pidUpdate_err(&Position_X_Speed,Position_X_SpeedErr,Interval_dt)/100.0f;
			Output = Math_fConstrain(Output,-2.4,2.4);
			Increas_Xspeed_Accumulat = Output;//增量式pid的输出的累加
			Target_Roll = (- Increas_Xspeed_Accumulat) * 10;
			RollAdd = Increas_Xspeed_Accumulat;  //记录每次角度的变化，传到上位机调试用
		}
		else
		{
			Increas_Xspeed_Accumulat = 0;
		  RollAdd = 0;
		}
		
		if((Target_Pitch > (int16_t)-30)&&(Target_Pitch < (int16_t)30))
		{
//			pidSetTarget_Measure(&Position_Hold,0,Ymove);
//			Position_MoveErr = Position_MoveErr
//			+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Position_Hold.merror , -3000.0f , 3000.0f) - Position_MoveErr);
//			Position_SpeedTarget = pidUpdate_err(&Position_Hold,Position_MoveErr,Interval_dt);
//			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-1000,+1000);
//			pidSetTarget_Measure(&Position_Speed,Position_SpeedTarget,Y_Speed);
//			Position_SpeedErr = Position_SpeedErr + //低通滤波。 2Hz
//    		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Position_Speed.merror, -1000.0f , 1000.0f) - Position_SpeedErr);
//			Output = pidUpdate_err(&Position_Speed,Position_SpeedErr,Interval_dt)/100;
//			Output = Math_fConstrain(Output,-10,+10);
//			Target_Pitch = (- Output) * 10;
//			PitchAdd = Output;  //记录每次角度的变化，传到上位机调试用
			
//			pidSetTarget_Measure(&Position_Y_Hold,0,Ymove);
//			Position_Y_MoveErr = Position_Y_MoveErr
//			+ (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_Y_Hold.merror , -1200.0f , 1200.0f) - Position_Y_MoveErr);
//			Position_SpeedTarget = pidUpdate_err(&Position_Y_Hold,Position_Y_MoveErr,Interval_dt);
//			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-200,+200);
//			pidSetTarget_Measure(&Position_Y_Speed,Position_SpeedTarget,Y_Speed);
//			Position_Y_SpeedErr = Position_Y_SpeedErr
//			  + (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_Y_Speed.merror , -500.0f , 500.0f) - Position_Y_SpeedErr);
//			Output = pidUpdate_err(&Position_Y_Speed,Position_Y_SpeedErr,Interval_dt)/10000.0;
//			Output = Math_fConstrain(Output,-0.05,0.05);
//			Increas_Yspeed_Accumulat += Output;//增量式pid的输出的累加
//			Increas_Yspeed_Accumulat = Math_fConstrain(Increas_Yspeed_Accumulat,-2.5,+2.5);
//			Target_Pitch = (Increas_Yspeed_Accumulat) * 10;
//			PitchAdd = Increas_Yspeed_Accumulat;  //记录每次角度的变化，传到上位机调试用

			pidSetTarget_Measure(&Position_Y_Hold,0,Ymove);
			Position_Y_MoveErr = Position_Y_MoveErr
			+ (Interval_dt / (15.9155e-3f + Interval_dt)) * (Math_fConstrain(Position_Y_Hold.merror , -1000.0f , 1000.0f) - Position_Y_MoveErr);
			Position_SpeedTarget = pidUpdate_err(&Position_Y_Hold,Position_Y_MoveErr,Interval_dt);
			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-200,+200);
			pidSetTarget_Measure(&Position_Y_Speed,Position_SpeedTarget,Y_Speed);
			Position_Y_SpeedErr = Position_Y_SpeedErr
			  + (Interval_dt / (15.9155e-3f + Interval_dt)) * (Math_fConstrain(Position_Y_Speed.merror , -250.0f , 250.0f) - Position_Y_SpeedErr);
			Output = pidUpdate_err(&Position_Y_Speed,Position_Y_SpeedErr,Interval_dt)/100.0f;
			Output = Math_fConstrain(Output,-2.4,2.4);
			Increas_Yspeed_Accumulat = Output;//增量式pid的输出的累加
			Target_Pitch = (Increas_Yspeed_Accumulat) * 10;
			PitchAdd = Increas_Yspeed_Accumulat;  //记录每次角度的变化，传到上位机调试用
		}
		else
		{
			Increas_Yspeed_Accumulat = 0;
		  PitchAdd = 0;
		}
	}
	else
	{
			Xmove = 0;
			Ymove = 0;
		  PitchAdd = 0;
			RollAdd = 0;
			Increas_Xspeed_Accumulat = 0;
			Increas_Yspeed_Accumulat = 0;
	    pidReset(&Position_X_Hold);
		  pidReset(&Position_X_Speed);
		  pidReset(&Position_Y_Hold);
		  pidReset(&Position_Y_Speed);
	    Quad_THR = (int16_t)(PWM_Input_CH3);
		  Position_X_MoveErr=0;
			Position_X_SpeedErr=0;
		  Position_Y_MoveErr=0;
			Position_Y_SpeedErr=0;
			last_call_us = 0;
			Target_Pitch=Target_Roll=0;
	}
		Roll_Pitch_Yaw_AnglePID(Target_Roll / 10.0f, Target_Pitch / 10.0f, Target_Yaw);
		
		PID_PITCH = PitchRate.PID_out;
		PID_ROLL = RollRate.PID_out;
		PID_YAW = YawRate.PID_out;
	
	PWM_Write_Motors();
}

/**************************实现函数********************************************
	*函数原型:	void Mode_ESC_Cal(void)
	*功　　能:  电调校准模式：启动时若CH3>1500则校准电调
*******************************************************************************/
void Mode_ESC_Cal(void)
{
	PWM_Output_ESC_Calibration(); //输出 3通道PWM输入的值.
	LED_Set_Blink(Red,150,150,3);  //提示正在设置电调行程
	if(Read_Mode() != Quad_Manual){
		Quadrotor_Mode = Quad_Manual;
	}
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Assignment(void)
	*功　　能:  任务模式
*******************************************************************************/
void Mode_Assignment(void)
{
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Landing(void)
	*功　　能:  降落模式
	这段代码写得特别乱，主要的功能就是实现在进入降落模式后飞行器一级一级以定高的方式降落。
	每到达一个高度就悬停一段时间，然后继续前往下一个高度。
	降落的最终目的高度是0.15m并不是0m。
	在到达目的高度后关闭油门。
*******************************************************************************/
float Land_targethigh = 0.0f;//0是一个判断条件，==0表示未开始降落
uint8_t High_Flag_IsLanded = 0;//是否已经到达地面；1：是，0：否。
void Mode_Landing(void)
{
    float altitude = MS5611_Altitude/100.0f;
    uint32_t Land_nowtime=micros();
    static uint32_t Land_lasttime=1;//不要初始化为0，lasttime==0是一个判断条件

    // Quad_THR = Default_Throttle;
    Quad_THR = (int16_t)(PWM_Input_CH3); //将通道3 输入做为油门量

    Get_Tartget_RPY(); //遥控信号转换为目标rpy
    Roll_Pitch_Yaw_AnglePID(Target_Roll / 10.0f, Target_Pitch / 10.0f, Target_Yaw);

	if(PWM_Input_CH3 > (int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10))//遥控器油门大于10% 定高才会起作用，防止误动作引起电机转动
	{
		//监测是否落地或近地
		if (altitude <= 0.15f)
	    {
			High_Flag_IsLanded = 1;
			Quad_THR = MINTHROTTLE;
			Height_PID_Reset();
	    }
		else if (altitude <= 0.3f)
			Quad_THR -= 50.0f;
		//是否已经落地了，如果已经落地，就不要再起来了，除非ch3又重新从10%以下开始往上提
	    if (High_Flag_IsLanded == 0)
	    {
		//阶段定高，到一个高度后1s后再前往下一个高度
			//如果计时的时间到了，则开始前往下一个目标高度，用lasttime是否==0来表示是否已经开始计时
			if ((Land_lasttime != 0) && (Land_nowtime - Land_lasttime >= 1500000))
			{
				Land_lasttime = 0;
				if(Land_targethigh==0)//Land_targethigh==0表示未开始自动降落
				    Land_targethigh = altitude;
				else
					Land_targethigh -= 0.8f;
				if (Land_targethigh < 0.1f)
					Land_targethigh = 0.1;
			}
			//如果已经到达目标高度并且没有开始计时则开始计时
			if ((altitude <= Land_targethigh + 0.1f) && (Land_lasttime == 0))
			{
				Land_lasttime = Land_nowtime;
			}
			Quad_THR += Height_PID(Land_targethigh);
		}
	    else
			Quad_THR = MINTHROTTLE;
	}
	else{
	    Height_PID_Reset();
	    High_Flag_IsLanded = 0; //ch3拉到10%以下可以清除落地标志
	    Land_lasttime = 1;      //不要初始化为0，lasttime==0是一个判断条件
	    Land_targethigh = 0.0f;
	}
	
	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;

	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
}

/**************************实现函数********************************************
*函数原型:	void Mode_Take_Of(void)
*功　　能:  起飞模式
*******************************************************************************/
void Mode_Take_Of(void)
{
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Auto_High(void)
	*功　　能:  定高模式
*******************************************************************************/
void Mode_Auto_High(void)
{
	// Quad_THR = Default_Throttle;//注意：如果油门用默认值的话，为了防止意外，所以需要ch3的输出大于一个值才定高，
								//因为一般人的逻辑是四轴失控了就马上调小油门而不是切模式
								//设置油门三个档位，最大档自动起飞，中档自动降落，小档关闭输出
	Quad_THR = (int16_t)PWM_Input_CH3; //将通道3 输入做为油门量
	
	Get_Tartget_RPY();//遥控信号的转换

	Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);

	if (PWM_Input_CH3 > (int16_t)(MINTHROTTLE + (MAXTHROTTLE - MINTHROTTLE) / 10))
	{ //遥控器油门大于10% 定高才会起作用，防止误动作引起电机转动
	    Quad_THR += Height_PID(Tartget_hight);
	}
	else
	{
	    Height_PID_Reset();
	}

	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;
	
	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
}
//------------------End of File----------------------------
