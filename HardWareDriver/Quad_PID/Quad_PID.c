#include "Quad_PID.h"
#include "pid.h"
#include "Quadrotor.h"
#include "IMU.h"

struct Quad_PID 
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
//定高相关变量
volatile float Increas_Output_Accumulat=0;//增量式pid的输出的累加
float Height_PID_Out = 0;

float GPS_PITCH=0,GPS_ROLL=0;

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

void Height_PID_Reset(void)
{
    pidReset(&Climb);
    pidReset(&AutoHigh_THR);
    pidReset(&Z_Speed);
    Increas_Output_Accumulat = 0;
    Height_PID_Out = 0;
}
/**************************实现函数********************************************
	*函数原型:		float Z_Speed_PID(float Speed)
	*功　　能:	  以固定速度Speed降落 单位米/秒
	输入：Speed：目标速度
	单位统一为cm和cm/s
*******************************************************************************/
float Z_Speed_PID(float Speed)
{	
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


//使用三级串联pid
	// Z_Speed是高度pid，AutoHigh_THR是速度pid，Climb是加速度pid
	// float ClimbTarget;
	// float THR_err;
	// static float alt_err = 0;
	// static float z_rate_error = 0;
	// static float z_accel_error = 0;
	// static uint32_t last_call_us = 0;

	// float Interval_dt = 0;
	// uint32_t now_time = micros();
	// if(now_time - last_call_us > 100000 ){ //超过100ms没有调用这个程序了。
	// 	z_rate_error = 0;
	// 	alt_err=0;
	// 	ClimbTarget = 0;
	// }

	// if(now_time - last_call_us < 20000 )return Climb.PID_out; //控制高度的更新速率为50hz

	// Interval_dt = (float)(now_time - last_call_us)/1000000.0f;//s
	// last_call_us = now_time;

	// pidSetKp(&Z_Speed, 0.7);
	// pidSetKi(&Z_Speed, 0.0);
	// pidSetKd(&Z_Speed, 0.0);
	// pidSetTarget_Measure(&Z_Speed,height*100.0f,MS5611_Altitude);
	// // if(Z_Speed.merror>3.0f||Z_Speed.merror<-3.0f)
	// {
	// 	alt_err = alt_err    //低通滤波。 2Hz
	// 		+ (Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Z_Speed.merror , -50.0f , 100.0f) - alt_err);
	// 	pidUpdate_err(&Z_Speed, alt_err, PID_dt);
	// }
	// pidSetTarget_Measure(&AutoHigh_THR,Z_Speed.PID_out,MS5611BA_Get_D());
	// // if(AutoHigh_THR.merror>5.0f||AutoHigh_THR.merror<-5.0f)
	// {
	// 	z_rate_error = z_rate_error + //低通滤波。 2Hz
	// 		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(AutoHigh_THR.merror, -100.0f , 100.0f) - z_rate_error);
	// 	pidUpdate_err(&AutoHigh_THR ,z_rate_error, PID_dt);//速度内环
	// }
	// pidSetTarget_Measure(&Climb,AutoHigh_THR.PID_out,(acc_vector - Config.ACC_z_zero) * 100.0f);
	// // if(Climb.merror>10.0f||Climb.merror<-10.0f)
	// {
	// 	z_accel_error = z_accel_error + //低通滤波。 2Hz
	// 		(Interval_dt / (0.0795775f + Interval_dt)) * (Math_fConstrain(Climb.merror, -200.0f , 200.0f) - z_accel_error);
	// 	pidUpdate_err(&Climb ,z_accel_error, PID_dt);//加速度内环

	// 	Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1550-Default_Throttle);
	// }
	// return Climb.PID_out;
//
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
	// if(now_time - last_call_us < 20000 )return Climb.PID_out; //控制高度的更新速率为50hz
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
	// pidUpdate_err(&Climb ,z_rate_error, ALT_Update_Interval);//速度内环
	// Climb.PID_out = Math_fConstrain(Climb.PID_out,1300-Default_Throttle,1550-Default_Throttle);
	// return Climb.PID_out;

	//如果要用增量式pid就用这三行
	THR_err = IncreasingPID(&Climb ,z_rate_error);
	// THR_err = Math_fConstrain(THR_err,-3.0f,+3.0f);
	Increas_Output_Accumulat+=THR_err;
  	// Increas_Output_Accumulat = Math_fConstrain(Increas_Output_Accumulat,1300-Default_Throttle,1550-Default_Throttle);
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
void GPS_Position_Hold(void)
{
    float Error[2], target_speed, speed_error[2], tmp;
    float p, i, d, out, dist = 0;
    float sin_yaw = sin(IMU_Yaw * 0.0174532925f); //机头的指向
    float cos_yaw = cos(IMU_Yaw * 0.0174532925f); //先将角度转成弧度 再计算cos
    uint8_t axis;

    dist = GPS_Distance(Latitude_GPS,
			Longitude_GPS,
			Home_Latitude,
			Home_Longitude);
    //计算四轴当前的位置  与家点的偏差。
    //Error[_X] 表示经度上的偏差
    //Error[_Y] 表示纬度上的偏差
    Error[_X] = (Longitude_GPS - Home_Longitude) * cos(((Home_Latitude + Latitude_GPS) / 2) * 0.0174532925f); // X Error
    Error[_Y] = Latitude_GPS - Home_Latitude;								      // Y Error

    for (axis = 0; axis < 2; axis++)
    {							    //计算目标角度
	target_speed = 0.11f * Error[axis];		    // calculate desired speed from lon error
	speed_error[axis] = target_speed - now_speed[axis]; // calc the speed error
	speed_error[axis] *= 10000000.0f;		    //以度为单位 显得偏差太小了，我们希望放大偏差
	//P
	p = Position_Hold.Kp * speed_error[axis];
	//I	   GPS_Period 为GPS位置更新的时间间隔
	Position_Hold_i[axis] += (speed_error[axis] + Error[axis]) * Position_Hold.Ki * GPS_Period;
	i = Math_fConstrain(Position_Hold_i[axis], -2000.0f, +2000.0f);
	//D
	tmp = (Error[axis] - Hold_last_error[axis]) / GPS_Period;
	//滤波
	d = Hold_last_d[axis] + (tmp - Hold_last_d[axis]) * (GPS_Period / (Hold_filter + GPS_Period));
	d *= Position_Hold.Kd;
	d = Math_fConstrain(d, -2000.0f, 2000.0f);

	Hold_last_error[axis] = Error[axis];
	Hold_last_d[axis] = tmp;

	out = p + i + d;

	//GPS_Hold_Angle[axis] = Math_fConstrain(out, -3000.0, +3000.0); 	// +-30 度
	GPS_Hold_Angle[axis] = Math_fConstrain(out, -1000.0f, +1000.0f); // +-10 度
    }

    //平滑处理
    Smooth_Ang[_X] += Math_fConstrain(Wrap_Ang(GPS_Hold_Angle[_X] - Smooth_Ang[_X]), -20.0, +20.0);
    Smooth_Ang[_Y] += Math_fConstrain(Wrap_Ang(GPS_Hold_Angle[_Y] - Smooth_Ang[_Y]), -20.0, +20.0);

    /*
  Smooth_Ang[_X] , Smooth_Ang[_Y] 对应的是以四轴指向正北的值。也就是YAW =0 时的值
  我们需要将它映射到四轴当前的航向上。
  */
    //ROLL
    GPS_ROLL = (Smooth_Ang[_X] * cos_yaw - Smooth_Ang[_Y] * sin_yaw) / 10.0f;
    //PITCH
    GPS_PITCH = (Smooth_Ang[_X] * sin_yaw + Smooth_Ang[_Y] * cos_yaw) / 10.0f;
	#if Captain_GCS
    UART1_ReportTarget( //xiang:注意这里给上位机上传了数据
						Home_Longitude * 1000000,
						Home_Latitude * 1000000,
						AutoHigh_THR.target * 10,
						dist * 10,
						GPS_ROLL,
						GPS_PITCH);
	#endif
}
