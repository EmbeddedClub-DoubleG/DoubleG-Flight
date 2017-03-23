#include "Quadrotor.h"
#include "common.h"
#include "Fmath.h"
#include "math.h"
#include "GPS.h"
#include "GCS_Protocol.h"

volatile uint8_t  Fly_Mode = default_quad; //飞行器的架构，默认十字四轴，见fly_config.h
volatile int16_t  Yaw_DIRECT = 1;
int16_t THROTTLE = MINTHROTTLE, PID_ROLL = 0, PID_PITCH = 0, PID_YAW = 0;
int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
volatile float PID_dt = 0;
volatile float Quad_Pitch,Quad_Roll,Quad_Yaw,Quad_THR=MINTHROTTLE;//update20161226:添加了Quad_Pitch,Quad_Roll,Quad_Yaw这几个变量，虽然没用到
float  Target_Roll,Target_Pitch,Target_Yaw,Tartget_hight = 0.4;//Target_Roll,Target_Pitch单位为 x/10=度
//xiang：这行代码是我注释掉的，因为IsAutoHigh这个东西在其他地方从来没有用到过
//volatile uint8_t IsAutoHigh = 0;  //是否开启定高。为0 不开启。为1 则开定高//没有用到过,所以注释掉//update20161226:修改注释
volatile uint8_t Quadrotor_Mode = Quad_Manual; 	//当前的飞行模式，手动，平衡，定点等
//int16_t tpitch,troll,tyaw,pidpitch,pidroll,pidyaw;
float GPS_PITCH=0,GPS_ROLL=0;

//xiang：主板B和任务模式相关变量
volatile uint8_t IsStart = 1;//是否刚开始或未开始任务模式。为1，是。为0，不是。
uint32_t StartTimeCnt;//进入任务模式的时间，用于在刚进入任务模式时在地面停留3秒
float GroundAltitude=0.0f;//启动时的高度，单位是米
float StartYaw=0;//启动时的航向
float RollAdd=0,PitchAdd=0;
uint32_t GetStartInfoCnt = 0;//多次记录启动高度和启动航向求平均值，这个值用来记录已经记录了多少次启动高度和启动航向
// float StayTopAltitude;//记录在任务模式时，飞行器飞到某个高度（一般是1.3米，因为任务要求1.3米，但是如果飞行器油门太大导致超过了1.3米，
						//则以此最高高度为要保持的高度）并且保持此高度的高度值

//xiang:主板B相关变量
#if BoardB_Enable//update20161226:新添加了条件编译BoardB
volatile uint8_t Command = Command_Stay;//从主板B发过来的命令
#endif

//xiang：定高相关变量
#define Default_Throttle 1470.0f//默认油门
//#define UnsafeSpeed -0.1f//距离地面UnsafeAltitude厘米下降速度<UnsafeSpeed米每秒则认为不安全
//#define UnsafeAltitude 50
volatile float Increas_Output_Accumulat=0;//增量式pid的输出的累加
volatile float Increas_Xspeed_Accumulat=0;//增量式pid的输出的累加
volatile float Increas_Yspeed_Accumulat=0;//增量式pid的输出的累加
float Height_PID_Out = 0;

#if Yingzhang_GCS
//update20170113
//上位机控制四轴目标角度相关变量
int8_t GCSControl_Forward = 0;
int8_t GCSControl_Backward = 0;
int8_t GCSControl_Leftward = 0;
int8_t GCSControl_Rightward = 0;
int8_t GCSControl_LeftRotate = 0;
int8_t GCSControl_RightRotate = 0;
#endif

// struct Quad_PID  PID_NULL[10];//update20161226:没用到,注释掉
struct Quad_PID 
    			// no_pid_temp,//update20161226:没用到,注释掉
				Stabilize_Roll,  // 横滚 PID
				RollRate,		  // 横滚 角速率PID
				Stabilize_Pitch,  // 府仰 PID
				PitchRate,		  // 府仰 角速率PID
				Stabilize_Yaw,	  //航向 PID
				YawRate,          //航向 角速率PID
				AutoHigh_THR,	  //定高 PID
				Climb,            //爬升 PID
				Z_Speed,		//
				Position_Hold,	  //位置 定点PID
				Position_Speed,	  //速度 PID
				Position_X_Hold,	  //位置 定点PID
				Position_X_Speed,	  //速度 PID
				Position_Y_Hold,	  //位置 定点PID
				Position_Y_Speed	  //速度 PID
				;	  

//int16_t Auto_High_PID(float TargetHigh,uint8_t isRate);
void Roll_Pitch_Yaw_RatePID(float Rate_roll,float Rate_pitch,float Rate_yaw);
void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw);
float Quadrotor_PID(float err,float Differ_in,struct Quad_PID *PID);
void PID_Reset_Integ(struct Quad_PID *PID);
float Get_Yaw_Error(float set,float currt);
//int16_t Pitch_Roll_PID(struct Quad_PID *PID,int16_t gyro);
//int16_t Thr_High_PID(void);
//int16_t Yaw_PID(float gyro);
void Position_Hold_Reset(void);
void GPS_Position_Hold(void);
float Z_Speed_PID(float Speed);
float Height_PID(float height);
void GetStartInfo(void);
unsigned char Read_Mode(void);
uint8_t Change_Mode(unsigned char);
void Led_Quad_Mode(void);
void Lock_Target_Yaw(void);
void Get_Tartget_RPY(void);
void Mode_Manual(void);
void Mode_Level_Lock(void);
void Mode_Hold_Position(void);
void Mode_ESC_Cal(void);
void Mode_Assignment(void);
void Mode_Landing(void);
void Mode_Take_Of(void);
void Mode_Auto_High(void);

//update20161227：已经不用Flag_Ultr_or_MS5611标识了，直接去用气压计的高度就可以了，如果超声波有效，气压计buffer里的数据全都是超声波的

/**************************实现函数********************************************
*函数原型:		void Quadrotor_Motor_Update(void)
*功　　能:	    电机转速更新。四轴的关键控制部分.
PWM输入定义：
	PWM_Input_CH1 目标横滚角度输入  1500us为0度
	PWM_Input_CH2 目标俯仰角度输入  1500us为0度
	PWM_Input_CH3 油门量输入
	PWM_Input_CH4 航向输入
	PWM_Input_CH5、PWM_Input_CH6 飞行模式控制
	油门量CH3大于1500us时，进入电调行程设置模式
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
	if(now_time < last_time){ //wu:是图麻烦所以这么写吗？
		last_time = now_time;
		return ;
	}
	PID_dt = (float)(now_time - last_time);
	PID_dt /= 1000000.0f;   //单位S
	last_time=now_time;
	
	mode=Read_Mode();//读取工作模式。通过PWM5、PWM6输入值判断
	Change_Mode(mode);//切换飞行模式
	Lock_Target_Yaw();//目标航向控制。当油门大于1100us时，认为用户希望起飞。那么此时的航向做为目标航向
	//判断当前飞行器是否安全，如果不安全则启动降落模式
	// if(MS5611_Altitude<=UnsafeAltitude&&Ultrasonic_Get_D()<=UnsafeSpeed)//判断高度和下降速度 MS5611_Altitude的单位是0.01m Ultrasonic_Get_D()的单位是米每秒
	// {
	// 	Quadrotor_Mode=Quad_Landing;
	// }
	
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
			//xiang:这段代码是我加的
			IsStart=1;
			break;
	}
}

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
	pidInit(&Z_Speed,     	Config.Climb_High_P,//update20170110
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
	pidSetLowPassEnable(&Z_Speed);//update20170110
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

	if((Fly_Mode == Y6)||(Fly_Mode == HEX6X)||(Fly_Mode == OCTOX8)){
	Set_PWMOuput_CH5(PWM_Input_CH3);
	Set_PWMOuput_CH6(PWM_Input_CH3);
	}
}

/**************************实现函数********************************************
	*函数原型:	int16_t Auto_High_PID(float Target,uint8_t isRate)
	*功　　能:  高度控制PID 
	TargetHigh  目标高度，单位 m[米]
	isRate 控制方式，1 为阻尼模式	限制爬升率
					0  高度锁定模式
	xiang:TargetHigh和Target的单位是米，而在所有的高度pid里，单位都是厘米
	xiang:这里也是串级pid，但是我没看懂
	xiang:在阻尼模式下tartget是爬升率（速度），在高度锁定模式下，tartget是高度
*******************************************************************************/
// int16_t Auto_High_PID(float Target,uint8_t isRate){
// 	static float ClimbTarget ;
// 	float alt_err , z_accel_meas;
// 	static float z_rate_error = 0;   //xiang:垂直速度,cm/s
// 	static float z_accel_error = 0;  //xiang:垂直加速度，cm/s2
// 	static uint32_t last_call_us = 0;
// 	float Interval_dt = 0;
// 	uint32_t now_time = micros();

// 	//需要先进行1G重力加速度值的采集。保存在AT45DB161
// 	if(Config.ACC_z_zero == 0.0f)return 0;  //没有初始化重力加速度值。退出
// 	if((Config.ACC_z_zero<8.8f)||(Config.ACC_z_zero>10.8f))return 0;//重力标定错误。应该在9.8附近

// 	if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
// 		z_rate_error = 0;
// 		z_accel_error = 0;
// 		ClimbTarget = 0;
// 		//xiang:应不应该清积分？
// 	}
// 	if(now_time - last_call_us < 20000 )return (int16_t)Climb.PID_out; //控制高度的更新速率为50hz
// 	Interval_dt = (float)(now_time - last_call_us)/1000000.0f;
// 	last_call_us = now_time;
	
// 	//xiang:这行代码是我改的
// 	//pidSetTarget_Measure(&AutoHigh_THR, Tartget_hight ,MS5611_Altitude/100.0f);
// 	pidSetTarget_Measure(&AutoHigh_THR, Target ,MS5611_Altitude/100.0f);
// 	if(isRate != 0){	 //为阻尼模式  阻止高度剧烈变化?
// 		ClimbTarget = Target; //爬升率=Target
// 		z_rate_error = 0;
// 	}
// 	else
// 	{
// 		alt_err = Target*100.0f - MS5611_Altitude; // 高度误差  cm
// 		alt_err = Math_fConstrain(alt_err , -80.0f , 80.0f); //限制爬升率  
// 		z_rate_error = z_rate_error    //低通滤波。 2Hz
// 		+ (Interval_dt / (0.0795775f + Interval_dt)) * ((alt_err - MS5611BA_Get_D()) - z_rate_error); 
// 		//高度外环
// 		ClimbTarget = pidUpdate_err(&AutoHigh_THR , 
// 								z_rate_error , // cm/s 
// 								ALT_Update_Interval);	//高度计更新间隔
// 		//xiang：这行代码是我改的
// 		//Math_fConstrain(ClimbTarget,-40.0f,+60.0f);
// 		ClimbTarget = Math_fConstrain(ClimbTarget,-40.0f,+60.0f);
// 	}

// 	//加速度计测得的垂直加速度。  cm/s^2
// 	z_accel_meas = (acc_vector - Config.ACC_z_zero) * 100.0f; //[m/s^2 -> cm/s^2]
// 	z_accel_error = z_accel_error + //低通滤波。 2Hz
// 			(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain((ClimbTarget - z_accel_meas), -3200.0f, 3200.0f) - z_accel_error);
// 	//PID计算油门量。
// 	pidUpdate_err(&Climb ,z_accel_error, ALT_Update_Interval);//速度内环
// 	Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1600-Default_Throttle);//xiang：佟源洋学长把这两个500改成250了;我改成了1200-Default_Throttle和1600-Default_Throttle

// 	return (int16_t)Climb.PID_out;
// }

//------------俯仰和横滚PID---------------------------------------
/**
	以十字模式 说明四轴角度平衡原理
	四轴抬头时(前高后低)，pitch > 0  前倾时Gyroy < 0  ,此时应该： 前电机减速，后电机加速，
	四轴低头时(前低后高)，pitch < 0  后倾时Gyroy > 0  ,此时应该： 前电机加速，后电机减速

	四轴右倾时(左高右低)，Roll > 0   右倾时Gyrox > 0  ,此时应该： 左电机减速，右电机加速
	四轴左倾时(左低右高)，Roll < 0	 左倾时Gyrox < 0	,此时应该： 左电机加速，右电机减速
**/
/**************************实现函数********************************************
	*函数原型:	void Roll_Pitch_Yaw_RatePID(float Rate_roll,float Rate_pitch,float Rate_yaw)
	*功　　能:  角速率控制PID
*******************************************************************************/
void Roll_Pitch_Yaw_RatePID(float Rate_roll,float Rate_pitch,float Rate_yaw){
	//ROLL
	pidSetTarget(&RollRate, Rate_roll*100.0f);
	pidUpdate(&RollRate ,IMU_GYROx*100.0f, PID_dt);
	RollRate.PID_out *= 0.1f;
	RollRate.PID_out = Math_fConstrain(RollRate.PID_out,-350.0f,+350.0f);  //限制控制PWM信号的幅度
	//PITCH
	pidSetTarget(&PitchRate, Rate_pitch*100.0f);
	pidUpdate(&PitchRate ,IMU_GYROy*100.0f , PID_dt);
	PitchRate.PID_out *= 0.1f;
	PitchRate.PID_out = Math_fConstrain(PitchRate.PID_out,-350.0f,+350.0f);  //限制控制PWM信号的幅度
	//YAW
	pidSetTarget(&YawRate, Rate_yaw*100.0f);
	pidUpdate(&YawRate ,IMU_GYROz*100.0f , PID_dt);
	YawRate.PID_out *= 0.1f;
	YawRate.PID_out = Math_fConstrain(YawRate.PID_out,-150.0f,+150.0f);  //限制控制PWM信号的幅度    
}

/**************************实现函数********************************************
	*函数原型:	void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw)
	*功　　能:  角度控制PID
	这里就是一个串级pid算法，角速度内环，角度外环，角度pid输出作为角速度pid输入（即目标值）
*******************************************************************************/

void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw){
	float RateTarget ,yaw_error;
	//ROLL
	pidSetTarget_Measure(&Stabilize_Roll, Angle_roll*100.0f,IMU_Roll*100.0f);	 //目标角度
	Stabilize_Roll.merror = Math_fConstrain(Stabilize_Roll.merror,-4500.0f,+4500.0f);
	RateTarget = pidUpdate(&Stabilize_Roll ,IMU_Roll*100.0f , PID_dt);
	pidSetTarget(&RollRate, RateTarget);
	pidUpdate(&RollRate ,IMU_GYROx*100.0f , PID_dt);
	RollRate.PID_out *= 0.1f;
	RollRate.PID_out = Math_fConstrain(RollRate.PID_out,-350.0f,+350.0f);  //限制控制PWM信号的幅度
	//PITCH
	pidSetTarget_Measure(&Stabilize_Pitch, Angle_pitch*100.0f,-IMU_Pitch*100.0f);
	Stabilize_Pitch.merror = Math_fConstrain(Stabilize_Pitch.merror,-4500.0f,+4500.0f);
	RateTarget = pidUpdate(&Stabilize_Pitch ,-IMU_Pitch*100.0f , PID_dt);
	pidSetTarget(&PitchRate, RateTarget);
	pidUpdate(&PitchRate ,IMU_GYROy*100.0f , PID_dt);
	PitchRate.PID_out *= 0.1f;
	PitchRate.PID_out = Math_fConstrain(PitchRate.PID_out,-350.0f,+350.0f);  //限制控制PWM信号的幅度
	//YAW
	pidSetTarget(&Stabilize_Yaw, Angle_yaw*100.0f);
	pidSetMeasured(&Stabilize_Yaw, IMU_Yaw*100.0f);	 
	yaw_error = -Get_Yaw_Error(Angle_yaw , IMU_Yaw);	//目标角度
	yaw_error *= 100.0f;
	yaw_error = Math_fConstrain(yaw_error,-4500.0f,+4500.0f);
	RateTarget = pidUpdate_err(&Stabilize_Yaw ,yaw_error, PID_dt);
	pidSetTarget(&YawRate, RateTarget);
	pidUpdate(&YawRate ,IMU_GYROz*100.0f , PID_dt);
	YawRate.PID_out *= 0.1f;
	YawRate.PID_out = Math_fConstrain(YawRate.PID_out,-150.0f,+150.0f);  //限制控制PWM信号的幅度
}

/**************************实现函数********************************************
	*函数原型:		float Z_Speed_PID(float Speed)
	*功　　能:	  以固定速度Speed降落 单位米/秒
	输入：Speed：目标速度
	单位统一为cm和cm/s
*******************************************************************************/
float Z_Speed_PID(float Speed)
{
//----使用增量式pid的代码---
	// float measured;
	// pidSetKp(&Z_Speed, 0.75f);	    //1.0
	// pidSetKi(&Z_Speed, 0.0f);	     //1.0
	// pidSetKd(&Z_Speed, 0.0f);	     //2.5
	// measured = MS5611BA_Get_D() / 100.0f; //  单位m/s
	// Z_Speed.merror = Speed - measured;
	// IncreasingPID(&Z_Speed, Z_Speed.merror);
	// Z_Speed.PID_out = Math_fConstrain(Z_Speed.PID_out, -50.0f, -15.0f);
	// return Z_Speed.PID_out;

//-----使用简单的pid的代码----
	// float measured ;	
	// measured = MS5611BA_Get_D();//  单位厘米
	// pidSetTarget_Measure(&Climb, Speed,measured);
	// //Climb.merror = Math_fConstrain(Climb.merror,-30.0f,+30.0f);
	// pidUpdate(&Climb ,measured , PID_dt);
	// Climb.PID_out = Math_fConstrain(Climb.PID_out,-150.0f,150.0f);
	// return Climb.PID_out;

//使用增量式PID的代码
	// static float Increas_Output_Accumulat=0;//增量式pid的输出的累加
	// pidSetTarget_Measure(&Climb,Speed,MS5611BA_Get_D()/100.0f);
	// Increas_Output_Accumulat += IncreasingPID(&Climb ,Climb.merror);
	// //Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,-150.0f,150.0f);
	// return Increas_Output_Accumulat;
	
//20161125从HeightPID裁剪的代码
	float ClimbTarget;
	float THR_err;
	static float z_rate_error = 0;
	static uint32_t last_call_us = 0;
	
	float Interval_dt = 0;
	uint32_t now_time = micros();
	if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
		z_rate_error = 0;
		ClimbTarget = 0;
	}
	
	//如果要位置式PID就用这行
	// if(now_time - last_call_us < 20000 )return Climb.PID_out; //控制高度的更新速率为50hz
	//如果要增量式PID就用这行
	if(now_time - last_call_us < 20000 )return Increas_Output_Accumulat; //控制高度的更新速率为50hz
	
	Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
	last_call_us = now_time;
	
	ClimbTarget = Speed*100.0f;
	ClimbTarget = Math_fConstrain(ClimbTarget,-200.0f,+200.0f);	
	pidSetTarget_Measure(&Climb,ClimbTarget,MS5611BA_Get_D());
	z_rate_error = z_rate_error + //低通滤波。 2Hz
		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Climb.merror, -200.0f , 200.0f) - z_rate_error);
	z_rate_error = Math_fConstrain(Climb.merror, -200.0f , 200.0f);
	//如果要用位置式pid就用这三行
	// pidUpdate_err(&Climb ,z_rate_error, ALT_Update_Interval);//速度内环
	// Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1550-Default_Throttle);
	// return Climb.PID_out;

	//如果要用增量式pid就用这三行
	THR_err = IncreasingPID(&Climb ,z_rate_error);
	//THR_err = Math_fConstrain(THR_err,-0.01f,+0.01f);	
	Increas_Output_Accumulat+=THR_err;
	Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,1300-Default_Throttle,1550-Default_Throttle);
	return Increas_Output_Accumulat;

}
/**************************实现函数********************************************
	*函数原型:	float Height_PID(float height)
	*功　　能:  高度控制PID 
	输入：height：目标高度m
	统一单位为cm和cm/s
*******************************************************************************/
float Height_PID(float height)
{
//营长的代码
    // static float PID_out = 0;
    // static float ClimbTarget;
    // float alt_err, z_accel_meas;
    // static float z_rate_error = 0;  // 垂直加速度，cm
    // static float z_accel_error = 0; //
    // static uint32_t last_call_us = 0;
    // float Interval_dt = 0;
    // uint32_t now = micros();

    // //以下都是我加的
    // static float alt[3] = {0};
    // float alt_current = 0;
    // static float last_PID_out = 0;

    // //需要先进行1G重力加速度值的采集。保存在AT45DB161
    // if (Config.ACC_z_zero == 0.0f)
	// 	return 0; //没有初始化重力加速度值。退出
    // if ((Config.ACC_z_zero < 8.8f) || (Config.ACC_z_zero > 10.8f))
	// 	return 0; //重力标定错误。应该在9.8附近

    // if (now - last_call_us > 100000)
    // { //超过100ms没有调用这个程序了。
	// 	z_rate_error = 0;
	// 	z_accel_error = 0;
	// 	ClimbTarget = 0;
	// 	alt[2] = alt[1] = alt[0] = 0;
	// 	PID_out = 0;
	// 	last_PID_out = 0;
    // }
    // if (now - last_call_us < 20000)
	// 	return (int16_t)Climb.PID_out; //控制高度的更新速率为50hz

    // Interval_dt = (float)(now - last_call_us) / 1000000.0f;
    // last_call_us = now;
    // pidSetTarget_Measure(&AutoHigh_THR, height, GetAltitude() / 100.0f);

    // alt[2] = alt[1];
    // alt[1] = alt[0];
    // alt[0] = GetAltitude();
    // alt_current = (alt[2] + alt[1] + alt[0]) / 3;
    // alt_err = height * 100.0f - alt_current;		 // 高度误差  cm
    // alt_err = Math_fConstrain(alt_err, -100.0f, 100.0f); //限制爬升率
    // //alt_err=alt_err*100;

    // AutoHigh_THR.merror = alt_err;
    // AutoHigh_THR.outP = AutoHigh_THR.Kp * alt_err;
    // AutoHigh_THR.Integrator += AutoHigh_THR.Ki * alt_err;
    // AutoHigh_THR.outI = Math_fConstrain(AutoHigh_THR.Integrator, -0.2f, +0.2f);
    // AutoHigh_THR.outD = AutoHigh_THR.Kd * (AutoHigh_THR.merror - AutoHigh_THR.last_error);
    // AutoHigh_THR.last_error = AutoHigh_THR.merror;
    // AutoHigh_THR.PID_out = AutoHigh_THR.outP + AutoHigh_THR.outI + AutoHigh_THR.outD;
    // ClimbTarget = AutoHigh_THR.PID_out;

    // //		ClimbTarget = pidUpdate_err(&AutoHigh_THR ,
    // //				            alt_err , // cm/s 改  原先是z_rate_error
    // //					    Interval_dt);	//高度计更新间隔 ALT_Update_Interval和Interval_dt   不知道用哪个，都试试吧
    // //z_accel_meas = (acc_vector - Config.ACC_z_zero) * 100.0f; //[m/s^2 -> cm/s^2]
    // ClimbTarget = Math_fConstrain(ClimbTarget, -0.1f, +0.1f);
    // PID_out += ClimbTarget;
    // PID_out = Math_fConstrain(PID_out, 0.0f, 100.0f); //油门基础值给1000；KP一定要给小。0.1左右吧，还得看看PIDupgrade_err里积分的上下行   0.15  0.005  1
    // //PID_out=last_PID_out+(PID_out-last_PID_out)*z_accel_meas/3000;
    // last_PID_out = PID_out; //写滤波时需要用到
    // //	ClimbTarget = Math_fConstrain(ClimbTarget,-80.0f,+120.0f);//加大
    // //	Climb.current = (Climb.merror-alt_err)/ALT_Update_Interval;//改
    // //	pidUpdate_err(&Climb ,ClimbTarget-Climb.current, ALT_Update_Interval);//改
    // //PID_out=PID_out/100.0;
    // Climb.PID_out = PID_out;
    // return (int16_t)Climb.PID_out; //改

    //使用增量式单级高度pid的代码
    // 	pidSetTarget_Measure(&AutoHigh_THR,height*100.0f,GetAltitude());
    // 	Increas_Output_Accumulat += IncreasingPID(&AutoHigh_THR ,AutoHigh_THR.merror);
    // 	Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,-150.0f,150.0f);
    // 	return Increas_Output_Accumulat;

// 使用位置式高度PID串联增量式/位置式速度PID的代码
	// // 电赛专用330机架适用pid（仅测试了增量式pid，位置式pid理论上一样）：
	// // 高度
	// // kp:0.6
	// // ki:0
	// // kd:0
	// // 速度
	// // kp:0.7
	// // ki:0.01
	// // kd:0.1
	// float ClimbTarget;
	// float THR_err;
	// static float alt_err = 0;
	// static float z_rate_error = 0;
	// static uint32_t last_call_us = 0;

	// float Interval_dt = 0;
	// uint32_t now_time = micros();
	// if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
	// 	z_rate_error = 0;
	// 	alt_err=0;
	// 	ClimbTarget = 0;
	// }

	// //如果要位置式PID就用这行
    // // if(now_time - last_call_us < 20000 )return Climb.PID_out; //控制高度的更新速率为50hz
	// //如果要增量式PID就用这行
	// if(now_time - last_call_us < 20000 )return Increas_Output_Accumulat; //控制高度的更新速率为50hz

	// Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
	// last_call_us = now_time;

	// // pidSetTarget_Measure(&AutoHigh_THR,height*100.0f,GetAltitude());//update20161227
	// pidSetTarget_Measure(&AutoHigh_THR,height*100.0f,MS5611_Altitude);
	// alt_err = alt_err    //低通滤波。 2Hz
	// 	+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(AutoHigh_THR.merror , -50.0f , 100.0f) - alt_err);
	// ClimbTarget = pidUpdate_err(&AutoHigh_THR , //climbtarget单位为cm/s
	// 							alt_err ,
	// 							ALT_Update_Interval);	//高度计更新间隔
	// ClimbTarget = Math_fConstrain(ClimbTarget,-60.0f,+60.0f);
	// // pidSetTarget_Measure(&Climb,ClimbTarget,GetZSpeed());//update20161227
	// pidSetTarget_Measure(&Climb,ClimbTarget,MS5611BA_Get_D());
	// z_rate_error = z_rate_error + //低通滤波。 2Hz
	// 	(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Climb.merror, -100.0f , 100.0f) - z_rate_error);

    // //如果要用位置式pid就用这三行
	// pidUpdate_err(&Climb ,z_rate_error, ALT_Update_Interval);//速度内环
	// Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1550-Default_Throttle);
	// // Height_PID_Out = Height_PID_Out + (Interval_dt / (0.0795775f + Interval_dt)) * Climb.PID_out;
	// return Climb.PID_out;

    //如果要用增量式pid就用这几行
	// THR_err = IncreasingPID(&Climb, z_rate_error, ALT_Update_Interval);
	// THR_err = Math_fConstrain(THR_err,-0.5f,+0.5f);
	// Increas_Output_Accumulat+=THR_err;
	// Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,1350-Default_Throttle,1600-Default_Throttle);
	// return Increas_Output_Accumulat;

//使用pid输出累加
    //	//static float Increas_Output_Accumulat=0;//增量式pid的输出的累加
    //	float ClimbTarget;
    //	float THR_err;
    //	static float alt_err = 0;
    //	static float z_rate_error = 0;
    //	static uint32_t last_call_us = 0;
    //
    //	float Interval_dt = 0;
    //	uint32_t now_time = micros();
    //	if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
    //		z_rate_error = 0;
    //		alt_err=0;
    //		ClimbTarget = 0;
    //	}
    //	if(now_time - last_call_us < 20000 )return Increas_Output_Accumulat; //控制高度的更新速率为50hz
    //	Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
    //	last_call_us = now_time;
    //
    //	pidSetTarget_Measure(&AutoHigh_THR,height*100.0f,MS5611_Altitude);
    //	alt_err = alt_err    //低通滤波。 2Hz
    //		+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(AutoHigh_THR.merror , -100.0f , 100.0f) - alt_err);
    //	ClimbTarget = pidUpdate_err(&AutoHigh_THR , //climbtarget单位为cm/s
    //								alt_err ,
    //								ALT_Update_Interval);	//高度计更新间隔
    //	ClimbTarget = Math_fConstrain(ClimbTarget,-100.0f,+100.0f);
    //	pidSetTarget_Measure(&Climb,ClimbTarget,MS5611BA_Get_D());
    //	z_rate_error = z_rate_error + //低通滤波。 2Hz
    //		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Climb.merror, -100.0f , 100.0f) - z_rate_error);

    //	pidUpdate_err(&Climb ,z_rate_error, ALT_Update_Interval);//速度内环
    //	Climb.PID_out = Math_fConstrain(Climb.PID_out,-20.0f,30.0f);
    //	Increas_Output_Accumulat+=Climb.PID_out;
    //	Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,1300-Default_Throttle,1550-Default_Throttle);
    //	return Increas_Output_Accumulat;
//使用三级串联pid
	//Z_Speed是高度pid，AutoHigh_THR是速度pid，Climb是加速度pid
//	float ClimbTarget;
//	float THR_err;
//	static float alt_err = 0;
//	static float z_rate_error = 0;
//	static float z_accel_error = 0;
//	static uint32_t last_call_us = 0;

//	float Interval_dt = 0;
//	uint32_t now_time = micros();
//	if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
//		z_rate_error = 0;
//		alt_err=0;
//		ClimbTarget = 0;
//	}

//    if(now_time - last_call_us < 20000 )return Climb.PID_out; //控制高度的更新速率为50hz

//	Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
//	last_call_us = now_time;

//	pidSetKp(&Z_Speed, 0.7);
//	pidSetKi(&Z_Speed, 0.0);
//	pidSetKd(&Z_Speed, 0.0);
//	pidSetTarget_Measure(&Z_Speed,height*100.0f,MS5611_Altitude);
//	// if(Z_Speed.merror>3.0f||Z_Speed.merror<-3.0f)
//	{
//		alt_err = alt_err    //低通滤波。 2Hz
//			+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Z_Speed.merror , -50.0f , 100.0f) - alt_err);
//		pidUpdate_err(&Z_Speed, alt_err, PID_dt);
//	}
//	pidSetTarget_Measure(&AutoHigh_THR,Z_Speed.PID_out,MS5611BA_Get_D());
//	// if(AutoHigh_THR.merror>5.0f||AutoHigh_THR.merror<-5.0f)
//	{
//		z_rate_error = z_rate_error + //低通滤波。 2Hz
//			(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(AutoHigh_THR.merror, -100.0f , 100.0f) - z_rate_error);
//		pidUpdate_err(&AutoHigh_THR ,z_rate_error, PID_dt);//速度内环
//	}
//	pidSetTarget_Measure(&Climb,AutoHigh_THR.PID_out,(acc_vector - Config.ACC_z_zero) * 100.0f);
//	// if(Climb.merror>10.0f||Climb.merror<-10.0f)
//	{
//		z_accel_error = z_accel_error + //低通滤波。 2Hz
//			(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Climb.merror, -200.0f , 200.0f) - z_accel_error);
//		pidUpdate_err(&Climb ,z_accel_error, PID_dt);//加速度内环

//		Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1550-Default_Throttle);
//	}
//	return Climb.PID_out;
    	float ClimbTarget;
    	float THR_err;
    	static float alt_err = 0;
    	static float z_rate_error = 0;
    	static uint32_t last_call_us = 0;

    	float Interval_dt = 0;
    	uint32_t now_time = micros();
    	if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
    		z_rate_error = 0;
    		alt_err=0;
    		ClimbTarget = 0;
    	}

    	//如果要位置式PID就用这行
    //	if(now_time - last_call_us < 20000 )return Climb.PID_out; //控制高度的更新速率为50hz
    	//如果要增量式PID就用这行
    	if(now_time - last_call_us < 20000 )return Increas_Output_Accumulat; //控制高度的更新速率为50hz

    	Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
    	last_call_us = now_time;

    	pidSetTarget_Measure(&AutoHigh_THR,height*100.0f,MS5611_Altitude);
    	alt_err = alt_err    //低通滤波。 2Hz
    		+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(AutoHigh_THR.merror , -150.0f , 100.0f) - alt_err);
    	ClimbTarget = pidUpdate_err(&AutoHigh_THR , //climbtarget单位为cm/s
    								alt_err ,
    								ALT_Update_Interval);	//高度计更新间隔  高度外环
    	ClimbTarget = Math_fConstrain(ClimbTarget,-200.0f,+200.0f); //高度外环
    	pidSetTarget_Measure(&Climb,ClimbTarget,MS5611BA_Get_D());
    	z_rate_error = z_rate_error + //低通滤波。 2Hz
    		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Climb.merror, -200.0f , 200.0f) - z_rate_error);

    //如果要用位置式pid就用这三行
    //	pidUpdate_err(&Climb ,z_rate_error, ALT_Update_Interval);//速度内环
    //	Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1550-Default_Throttle);
    //	return Climb.PID_out;

    //如果要用增量式pid就用这三行
    	THR_err = IncreasingPID(&Climb ,z_rate_error);
    //	THR_err = Math_fConstrain(THR_err,-3.0f,+3.0f);
    	Increas_Output_Accumulat+=THR_err;
//   	Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,1300-Default_Throttle,1550-Default_Throttle);
			Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,1300-Default_Throttle,1580-Default_Throttle);
    	return Increas_Output_Accumulat;
}

//-----------------------------------------------------------------------------
/// Low pass filter cut frequency for derivative calculation.
	static const float Hold_filter = 7.9577e-3;
		// Examples for _filter:
		// f_cut = 10 Hz -> _filter = 15.9155e-3
		// f_cut = 15 Hz -> _filter = 10.6103e-3
		// f_cut = 20 Hz -> _filter =  7.9577e-3
		// f_cut = 25 Hz -> _filter =  6.3662e-3
		// f_cut = 30 Hz -> _filter =  5.3052e-3
	float Position_Hold_i[2]={0.0f,0.0f};
	float Hold_last_error[2]={0.0f,0.0f};
	float Hold_last_d[2]={0.0f,0.0f};
	float GPS_Hold_Angle[2]={0.0f,0.0f};
	float Smooth_Ang[2]={0.0f,0.0f};

	float Wrap_Ang(float ang) {
	if (ang > 18000)  ang -= 36000;
	if (ang < -18000) ang += 36000;
	return ang;
	}
/**************************实现函数********************************************
	*函数原型:	void Position_Hold_Reset(void)
	*功　　能:  
*******************************************************************************/
void Position_Hold_Reset(void) {
	uint8_t	axis ;
	for (axis = 0; axis < 2; axis ++) {
	Position_Hold_i[axis] = 0.0f;
	Hold_last_error[axis] = 0.0f;
	Hold_last_d[axis] = 0.0f;
	GPS_Hold_Angle[axis] = 0.0f;
	Smooth_Ang[axis] = 0.0f;
	}
	GPS_PITCH = 0.0f;
	GPS_ROLL = 0.0f;
}
/**************************实现函数********************************************
	*函数原型:	void GPS_Position_Hold(void)
	*功　　能:  
*******************************************************************************/
void GPS_Position_Hold(void) {
	float Error[2] , target_speed , speed_error[2] , tmp;
	float p , i , d , out , dist=0;
	float sin_yaw = sin(IMU_Yaw * 0.0174532925f);	//机头的指向
	float cos_yaw = cos(IMU_Yaw * 0.0174532925f);	//先将角度转成弧度 再计算cos
	uint8_t	axis ;
  
	dist = GPS_Distance(Latitude_GPS,
						Longitude_GPS,
						Home_Latitude,
						Home_Longitude
						);  
	//计算四轴当前的位置  与家点的偏差。  
	//Error[_X] 表示经度上的偏差
	//Error[_Y] 表示纬度上的偏差 
	Error[_X] = (Longitude_GPS - Home_Longitude) * cos(((Home_Latitude + Latitude_GPS)/2)* 0.0174532925f);  // X Error
	  Error[_Y] = Latitude_GPS - Home_Latitude ; // Y Error

	for (axis = 0; axis < 2; axis ++) {	//计算目标角度
	target_speed = 0.11f * Error[axis];	// calculate desired speed from lon error
	speed_error[axis] = target_speed - now_speed[axis];	// calc the speed error
	  speed_error[axis] *= 10000000.0f	;  //以度为单位 显得偏差太小了，我们希望放大偏差
	//P
	p = Position_Hold.Kp * speed_error[axis];
	//I	   GPS_Period 为GPS位置更新的时间间隔
	Position_Hold_i[axis] += (speed_error[axis] + Error[axis]) * Position_Hold.Ki * GPS_Period;
	i = Math_fConstrain(Position_Hold_i[axis] , -2000.0f , +2000.0f);
	  //D
	tmp = (Error[axis] - Hold_last_error[axis]) / GPS_Period;
	//滤波
	d = Hold_last_d[axis] + (tmp - Hold_last_d[axis])*(GPS_Period / ( Hold_filter + GPS_Period));
	d *= Position_Hold.Kd;
	d = Math_fConstrain(d, -2000.0f, 2000.0f);

	Hold_last_error[axis] = Error[axis] ; 
	Hold_last_d[axis] = tmp;

	out  = p + i + d;

	//GPS_Hold_Angle[axis] = Math_fConstrain(out, -3000.0, +3000.0); 	// +-30 度
	GPS_Hold_Angle[axis] = Math_fConstrain(out, -1000.0f, +1000.0f); 	// +-10 度	
  }

  //平滑处理
  Smooth_Ang[_X] += Math_fConstrain(Wrap_Ang(GPS_Hold_Angle[_X]- Smooth_Ang[_X]) ,-20.0,+20.0);
  Smooth_Ang[_Y] += Math_fConstrain(Wrap_Ang(GPS_Hold_Angle[_Y]- Smooth_Ang[_Y]) ,-20.0,+20.0);

  /*
  Smooth_Ang[_X] , Smooth_Ang[_Y] 对应的是以四轴指向正北的值。也就是YAW =0 时的值
  我们需要将它映射到四轴当前的航向上。
  */
  //ROLL
  GPS_ROLL = (Smooth_Ang[_X]*cos_yaw - Smooth_Ang[_Y]*sin_yaw) / 10.0f;
  //PITCH
  GPS_PITCH = (Smooth_Ang[_X]*sin_yaw + Smooth_Ang[_Y]*cos_yaw) / 10.0f;
#if Captain_GCS
  UART1_ReportTarget(//xiang:注意这里给上位机上传了数据
					  Home_Longitude*1000000,
					Home_Latitude*1000000,
					AutoHigh_THR.target*10,
					dist *10,
					GPS_ROLL ,
					GPS_PITCH 
					  );
#endif
}
/**************************实现函数********************************************
	*函数原型:	void GetStartInfo(void)
	*功　　能:  获取当前的1.高度2.航向信息3.遥控器输入与过去测量数据的平均值的带权平均值
*******************************************************************************/
void GetStartInfo(void)
{
	GroundAltitude = (GroundAltitude * GetStartInfoCnt + MS5611_Altitude) / (GetStartInfoCnt + 1 );//启动时记录地面高度
	//保持当前航向角的值和已经记录的启动航向角的平均值在同一个半圆
	while(IMU_Yaw - StartYaw > 180.0f)
	{
		StartYaw+=360.0f;
	}
	while(IMU_Yaw - StartYaw < -180.0f)
	{
		StartYaw-=360.0f;
	}
	StartYaw = (StartYaw * GetStartInfoCnt + IMU_Yaw) / (GetStartInfoCnt + 1 );//记录启动时的航向，在执行任务时，始终保持这个航向
	GetStartInfoCnt++;
}
/**************************实现函数********************************************
	*函数原型:	unsigned char Read_Mode(void)
	*功　　能:  读取PWM5\PWM6输入值，确定飞控的工作模式
	输出参数：飞行模式标识
*******************************************************************************/
unsigned char Read_Mode(void)
{
 	// if(PWM_Input_CH6 < (int16_t)(PWM_Input_Offset-100))//CH6小于 1400us
	// {
	// 	if(PWM_Input_CH5 < (int16_t)(PWM_Input_Offset-100))//CH5小于 1400us
	// 		//左下右上：
	// 		return Quad_Auto_High;//定高模式
	// 	else if(PWM_Input_CH5 >(int16_t)(PWM_Input_Offset+100))//CH5大于 1600us
	// 		//左上右上：
	// 		return Quad_Take_Of;//起飞模式
	// }
	// else if(PWM_Input_CH6 >(int16_t)(PWM_Input_Offset+100))//CH6大于 1600us
	// {
	// 	if(PWM_Input_CH5 < (int16_t)(PWM_Input_Offset-100))//CH5小于 1400us
	// 		//左下右下：
	// 		return 	Quad_Landing ;//降落模式
	// 	else if(PWM_Input_CH5 >(int16_t)(PWM_Input_Offset+100))//CH5大于 1600us
	// 		//左上右下：
	// 		return Quad_Level_Lock;//平衡模式
	// }
	// return Quad_Manual;
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
/**************************实现函数********************************************
	*函数原型:	uint8_t Change_Mode(unsigned char)
	*功　　能:  切换飞行模式
	输入参数：待决定的飞行模式标识
	输出参数：已决定的飞行模式标识
*******************************************************************************/
uint8_t Change_Mode(unsigned char mode)
{
	//  if((Quadrotor_Mode != mode)&&(Quadrotor_Mode != Quad_ESC_Cal)&&(Quadrotor_Mode != Quad_Landing))//记住，这里如果从landing模式切换出去记得清pid
	if((Quadrotor_Mode != mode)&&(Quadrotor_Mode != Quad_ESC_Cal))
	{
		if(mode==Quad_Take_Of && ( Quadrotor_Mode==Quad_Take_Of||Quadrotor_Mode==Quad_Landing||Quadrotor_Mode==Quad_Auto_High)){
			return Quadrotor_Mode;//如果原先的飞行模式是任务模式而现在读取到的模式不是任务模式，则启动降落模式
		}else
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
	*函数原型:	void Get_Tartget_RPY(void)
	*功　　能:  遥控信号的转换。读取遥控器信号，转换为相应的roll、pitch、yaw
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
	IsStart=1;//表示未开始任务模式
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
	IsStart=1;//表示未开始任务模式
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Hold_Position(void)
	*功　　能:  定点模式
*******************************************************************************/
void Mode_Hold_Position(void)
{
	
//	Get_Tartget_RPY();//遥控信号的转换
//	Quad_THR = (int16_t)(PWM_Input_CH3); //将通道3 输入做为油门量
//	//-------------定点飞行------------	
//	//xiang：定点飞行模式除了添加了这一段代码，其他的和平衡模式一样；而这一段代码主要和GPS有关。
//	if(Home_Ready)
//	{    //是否保存了家点。Home_Ready变量在GPS.c中定义
//		if(GPS_Update)
//		{	//GPS数据有更新。
//			GPS_Position_Hold();  //重新计算定点值。
//			GPS_Update = 0;			
//		}
//		 Target_Roll += GPS_ROLL; //如果没有安装GPS，程序不会运行到这里
//		 Target_Pitch += GPS_PITCH;
//	}
//	else
//	{
//		 Position_Hold_Reset();
//	}
//	//--------通过PID计算PWM输出-------
//	if(PWM_Input_CH3 > (int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10))
//	{
//		//遥控器油门大于10% 定高才会起作用，防止误动作引起电机转动
//		Quad_THR += Height_PID(Tartget_hight);
//	}
//	else
//	{
//		pidReset(&Climb);
//		pidReset(&AutoHigh_THR);
//		pidReset(&Z_Speed);		
//	}
//	Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
//	PID_PITCH = PitchRate.PID_out;
//	PID_ROLL = RollRate.PID_out;
//	PID_YAW = YawRate.PID_out;
//	THROTTLE = Quad_THR;
//	//写输出到比较器  改写PWM输出脉宽值

//	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
//	IsStart=1;//表示未开始任务模式
	
	//营长:以下代码是我用光流模块写的悬停,以上注释部分为原GPS定点
	
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
	    pidReset(&Climb);
	    pidReset(&AutoHigh_THR);
		  pidReset(&Z_Speed);		
	    Increas_Output_Accumulat = 0;
	    Height_PID_Out = 0;
	    Quad_THR = (int16_t)(PWM_Input_CH3);
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
			+ (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_X_Hold.merror , -1000.0f , 1000.0f) - Position_X_MoveErr);
			Position_SpeedTarget = pidUpdate_err(&Position_X_Hold,Position_X_MoveErr,Interval_dt);
			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-200,+200);
			pidSetTarget_Measure(&Position_X_Speed,Position_SpeedTarget,X_Speed);
			Position_X_SpeedErr = Position_X_SpeedErr
			+ (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_X_Speed.merror , -250.0f , 250.0f) - Position_X_SpeedErr);
			Output = pidUpdate_err(&Position_X_Speed,Position_X_SpeedErr,Interval_dt)/100.0;
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
			+ (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_Y_Hold.merror , -1000.0f , 1000.0f) - Position_Y_MoveErr);
			Position_SpeedTarget = pidUpdate_err(&Position_Y_Hold,Position_Y_MoveErr,Interval_dt);
			Position_SpeedTarget = Math_fConstrain(Position_SpeedTarget,-200,+200);
			pidSetTarget_Measure(&Position_Y_Speed,Position_SpeedTarget,Y_Speed);
			Position_Y_SpeedErr = Position_Y_SpeedErr
			  + (Interval_dt / (15.9155e-3 + Interval_dt)) * (Math_fConstrain(Position_Y_Speed.merror , -250.0f , 250.0f) - Position_Y_SpeedErr);
			Output = pidUpdate_err(&Position_Y_Speed,Position_Y_SpeedErr,Interval_dt)/100.0;
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
	IsStart=1;//表示未开始任务模式
}

/**************************实现函数********************************************
	*函数原型:	void Mode_ESC_Cal(void)
	*功　　能:  电调校准模式：启动时若CH3>1500则校准电调
*******************************************************************************/
void Mode_ESC_Cal(void)
{
	PWM_Output_ESC_Calibration(); //输出 3通道PWM输入的值.
	//LED_Set_Blink(Red,150,150,3);  //提示正在设置电调行程
	if(Read_Mode() != Quad_Manual){
		Quadrotor_Mode = Quad_Manual;
	}
	IsStart=1;//表示未开始任务模式
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Assignment(void)
	*功　　能:  任务模式
*******************************************************************************/
void Mode_Assignment(void)
{
	// if(IsStart==1)
	// {
	// 	StartTimeCnt = micros();
	// 	IsStart=0;
	// 	GetStartInfoCnt=0;
	// }
	// else 
	// {
	// 	if(micros() - StartTimeCnt < 3000000)//时间的单位是us，当开始进入任务模式时，在原地静止3秒
	// 	{
	// 		GetStartInfo();
	// 		LEDRed_ON();//在这静止的3秒中，红灯常亮
	// 	}
	// 	else
	// 	{
	// 		//任务模式没有遥控器的ch1~4的输入，所有的东西有传感器和默认输入来决定	
	// 		while(StartYaw >180.0f)//转[-180.0,+180.0]
	// 			{StartYaw = StartYaw-360.0f;}
	// 		while(StartYaw <-180.0f)
	// 			{StartYaw = 360.0f + StartYaw;}
	// 		Target_Yaw = StartYaw; //将当前的航向做为目标航向
	// 		Quad_THR = Default_Throttle; //取默认油门
	// 		LEDRed_OFF();
	// 		BoardB_SendChar(Message_Start);//进入任务模式并且在3秒延时后，给主板B发消息
	// 		switch(Command)//执行当前命令
	// 		{
	// 			case Command_Stay:
	// 				//Quad_THR += Auto_High_PID(MS5611_Altitude , 0);
	// 				pidReset(&Climb);
	// 				Target_Roll = 0;
	// 				Target_Pitch = 0;
	// 				Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
	// 				//Roll_Pitch_Yaw_RatePID(0,0,0);
	// 				PID_PITCH = PitchRate.PID_out;
	// 				PID_ROLL = RollRate.PID_out;
	// 				PID_YAW = YawRate.PID_out;
	// 				THROTTLE = Quad_THR;
	// 				PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	// 				break;//Command_Stay break
				
	// 			case Command_Up:
	// 				//target_hight和Staytopaltitude和autohighpid的单位都是米
	// 				Tartget_hight=1.3f;
	// 				if(MS5611_Altitude/100.0f >= Tartget_hight)
	// 				{
	// 					StayTopAltitude = MS5611_Altitude/100.0f;
	// 					Quad_THR += Auto_High_PID(StayTopAltitude , 0);
	// 				}else
	// 				{
	// 					Quad_THR += Auto_High_PID(Tartget_hight , 0);
	// 				}
	// 				Target_Roll = 0;
	// 				Target_Pitch = 0;
	// 				Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
	// 				PID_PITCH = PitchRate.PID_out;
	// 				PID_ROLL = RollRate.PID_out;
	// 				PID_YAW = YawRate.PID_out;
	// 				THROTTLE = Quad_THR;
	// 				PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	// 				break;//Command_Up break
				
	// 			case Command_Down:
	// 				Tartget_hight = GroundAltitude;
	// 				if(MS5611_Altitude <= Tartget_hight)
	// 				{
	// 					//Quad_THR += Auto_High_PID(MS5611_Altitude , 0);
	// 					pidReset(&Climb);
	// 				}else
	// 				{
	// 					Quad_THR += Auto_High_PID(Tartget_hight , 1);
	// 				}
	// 				//Target_Roll = 0;
	// 				//Target_Pitch = 0;
	// 				//Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
	// 				Roll_Pitch_Yaw_RatePID(0,0,0);
	// 				PID_PITCH = PitchRate.PID_out;
	// 				PID_ROLL = RollRate.PID_out;
	// 				PID_YAW = YawRate.PID_out;
	// 				THROTTLE = Quad_THR;
	// 				PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	// 				break;//Command_Up break
					
	// 			case Command_Forward:					
	// 				Target_Roll = 0;
	// 				Target_Pitch = 0;

	// 		}//switch(Command)//执行当前命令

	// 	}//micros() - StartTimeCnt >= 3000000
	// }//IsStart==0
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Landing(void)
	*功　　能:  降落模式
*******************************************************************************/
float Land_targethigh = 0.0f;//0是一个判断条件，==0表示未开始降落
uint8_t High_Flag_IsLanded = 0;//是否已经到达地面；1：是，0：否。
void Mode_Landing(void)
{//TODO 解决气压计的问题
    // float altitude = GetAltitude()/100.0f;//update20161227
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
			pidReset(&Climb);
			pidReset(&AutoHigh_THR);
			pidReset(&Z_Speed);		
			Increas_Output_Accumulat = 0;
			Height_PID_Out = 0;
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
		pidReset(&Climb);
		pidReset(&AutoHigh_THR);
		pidReset(&Z_Speed);		
		Increas_Output_Accumulat=0;
		Height_PID_Out = 0;
		Quad_THR = (int16_t)(PWM_Input_CH3);
		High_Flag_IsLanded = 0;//ch3拉到10%以下可以清除落地标志
		Land_lasttime=1;//不要初始化为0，lasttime==0是一个判断条件
		Land_targethigh = 0.0f;
	}
	
	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;

	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	IsStart=1;//表示未开始任务模式
}

/**************************实现函数********************************************
*函数原型:	void Mode_Take_Of(void)
*功　　能:  起飞模式
*******************************************************************************/
void Mode_Take_Of(void)
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
	    pidReset(&Climb);
	    pidReset(&AutoHigh_THR);
		pidReset(&Z_Speed);		
	    Increas_Output_Accumulat = 0;
	    Height_PID_Out = 0;
	    Quad_THR = (int16_t)(PWM_Input_CH3);
	}

	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;
	
	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	IsStart=1;//表示未开始任务模式
}

/**************************实现函数********************************************
	*函数原型:	void Mode_Auto_High(void)
	*功　　能:  定高模式
	这个函数的思想是用ch3当作目标速度，ch3为1500时速度为0
*******************************************************************************/
float Autohigh_Targetspeed = 0.0f;
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
	    Quad_THR += Height_PID(MS5611_Altitude/100.0f);
	}
	else
	{
	    pidReset(&Climb);
	    pidReset(&AutoHigh_THR);
		pidReset(&Z_Speed);		
	    Increas_Output_Accumulat = 0;
	    Height_PID_Out = 0;
	    Quad_THR = (int16_t)(PWM_Input_CH3);
	}

	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;
	
	PWM_Write_Motors(); //写输出到PWM通道  {调用PWM_Output.c的子程序}
	IsStart=1;//表示未开始任务模式
}
//------------------End of File----------------------------
