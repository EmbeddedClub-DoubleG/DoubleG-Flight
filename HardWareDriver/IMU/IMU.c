/* IMU.c file
占用STM32 资源：
1. 使用Tim7定时器 产生us级的系统时间

功能：
姿态解算 IMU
将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
------------------------------------
*/

#include "IMU.h"

const static float Quad_Offset_Roll = 0.0f;//由于电路板安装时的问题，可能导致组装好飞行器后mpu6050不是水平的，所以用这两个偏置来抵消，这两个数值是要自己试飞的时候一点点调的
const static float Quad_Offset_Pitch = 0.0f;//这个不用管单位，去看上位机姿态解算的值大概估算一下，慢慢调，Quad_Offset_Roll数值变大机身往左方偏，Quad_Offset_Pitch数值变大往前方偏

volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float integralFBx,integralFBy,integralFBz;
volatile float q0, q1, q2, q3; // 全局四元数
volatile float qa0, qa1, qa2, qa3;
volatile float integralFBhand,handdiff;
volatile double halftime ;
volatile uint32_t IMU_lastUpdateTime; // 采样周期计数 单位 us
volatile uint16_t sysytem_time_ms=0;
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;//当前解算的姿态
volatile float  IMU_GYROx, IMU_GYROy, IMU_GYROz;
volatile unsigned char IMU_inited = 0;
volatile uint16_t imu_clce =0;
float acc_vector = 0;  //当前加速度感应到的力合  M/S^2
//update20170110:增加加速度计累加计算速度，速度累加计算位移
volatile float Motion_Accx = 0, Motion_Accy = 0, Motion_Accz = 0;
volatile float Motion_Velocity_X = 0, Motion_Velocity_Y = 0, Motion_Velocity_Z = 0;
volatile float Position_X = 0, Position_Y = 0, Position_Z = 0;
//计算xyz方向加速度的均值滤波
#define MOVAVG_SIZE  10
// float Acc_buffer_X[MOVAVG_SIZE]={0};
// float Acc_buffer_Y[MOVAVG_SIZE]={0};
// float Acc_buffer_Z[MOVAVG_SIZE]={0};
// uint32_t Acc_index = 0;//accxyz三个buffer共用一个index，每次添加新值都一起添加
float Motion_Acc_buffer_X[MOVAVG_SIZE]={0};
float Motion_Acc_buffer_Y[MOVAVG_SIZE]={0};
float Motion_Acc_buffer_Z[MOVAVG_SIZE]={0};
uint32_t Motion_Acc_index = 0;
float Acc_vector_buffer[MOVAVG_SIZE] = {0}; //update20170314:计算合加速度的均值滤波
uint32_t Acc_vector_index = 0;
uint32_t Acc_lastUpdateTime = 0;
uint32_t Velocity_lastUpdateTime = 0;
uint32_t Position_lastUpdateTime = 0;
float Motion_Velocity_dt = 0;
float Motion_Position_dt = 0;

void Motion_Acc_NewValue(float motion_accx,float motion_accy,float motion_accz)
{	 
	Motion_Acc_buffer_X[Motion_Acc_index] = motion_accx;
	Motion_Acc_buffer_Y[Motion_Acc_index] = motion_accy;
	Motion_Acc_buffer_Z[Motion_Acc_index] = motion_accz;
	Motion_Acc_index++;
	if(Motion_Acc_index>=MOVAVG_SIZE)
	{
		Motion_Acc_index -= MOVAVG_SIZE;
	}
}
//添加一个新的值到 加速度队列 进行滤波
// void Acc_NewValue(float accx,float accy,float accz)
// {	 
// 	Acc_buffer_X[Acc_index] = accx;
// 	Acc_buffer_Y[Acc_index] = accy;
// 	Acc_buffer_Z[Acc_index] = accz;
// 	Acc_index++;
// 	if(Acc_index>=MOVAVG_SIZE)
// 	{
// 		Acc_index -= MOVAVG_SIZE;
// 	}
// }
void AccVector_NewValue(float accvector)
{
	Acc_vector_buffer[Acc_vector_index] = accvector;
	Acc_vector_index++;
	if(Acc_vector_index>=MOVAVG_SIZE)
	{
		Acc_vector_index -= MOVAVG_SIZE;
	}
}

//读取队列 的平均值
float Acc_GetAvg(float *buff)
{
	float sum = 0.0;
	int i;
	for (i = 0; i < MOVAVG_SIZE; i++)
	{
		sum += buff[i];
	}
	return (sum / MOVAVG_SIZE);
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关	
				初始化各个传感器
				初始化四元数
				将积分清零
				更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void IMU_init(void)
{
    int i;
    LSM303_Initial();
    MPU6500_initialize();
    delay_ms(50);
    MPU6500_initialize();
    LSM303_Initial();
    delay_ms(50);
    MPU6500_initialize();
    LSM303_Initial();
    MS561101BA_init();
    // initialize quaternion
    q0 = 1.0f; //初始化四元数
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    qa0 = 1.0f; //初始化四元数
    qa1 = 0.0f;
    qa2 = 0.0f;
    qa3 = 0.0f;
    exInt = 0.0;
    eyInt = 0.0;
    ezInt = 0.0;

    integralFBx = 0.0;
    integralFBy = 0.0;
    integralFBz = 0.0;
    IMU_lastUpdateTime = micros(); //更新时间
    //IMU_nowtime = micros();
    Position_lastUpdateTime = Velocity_lastUpdateTime = Acc_lastUpdateTime = IMU_lastUpdateTime = micros(); //更新时间//update20170314
    for (i = 0; i < MOVAVG_SIZE; i++)
    {
		Acc_vector_buffer[i] = 9.65f;
	}
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
#define new_weight 0.5f
#define old_weight 0.5f

void IMU_getValues(float * values)
{  
	int16_t accgyroval[9];
	static float lastacc[3]= {0,0,0};
	int i;
	//读取加速度和陀螺仪的当前ADC
		//LSM303_readAcc(&accgyroval[0]);	  //加速度
	//MAX21000_readGyro(&accgyroval[3]); //陀螺仪
	LSM303_readMag(&accgyroval[6]);	//读取磁力计的ADC值
	MPU6500_readGyro_Acc(&accgyroval[3],&accgyroval[0]);
	for(i = 0; i<6; i++)
	{
		if(i < 3)
		{
			values[i] = (float) accgyroval[i] * new_weight +lastacc[i] * old_weight ;
			lastacc[i] = values[i];
		}else{
			values[i] = ((float) accgyroval[i]) / 16.4f; //转成度每秒
			//这里已经将量程改成了 2000度每秒  15.0 对应 1度每秒
		}
	}
	values[6] = (float)accgyroval[6];
	values[7] = (float)accgyroval[7];
	values[8] = (float)accgyroval[8];
	IMU_GYROx = accgyroval[3] / 16.4f;
	IMU_GYROy = accgyroval[4] / 16.4f;
	IMU_GYROz = accgyroval[5] / 16.4f;
}


/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.03f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	volatile float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez,halfT;
	float temp0,temp1,temp2,temp3;
	float temp;
	// 先把这些用得到的值算好
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	//计算dt和dt/2
	uint32_t nowtime = micros();  //读取时间
	if(nowtime < IMU_lastUpdateTime){ //定时器溢出过了。
		halfT =  ((float)(nowtime + (0xffffffff- IMU_lastUpdateTime)) / 2000000.0f);	
		IMU_lastUpdateTime = nowtime;
		//return ;
	}else{
		halfT =  ((float)(nowtime - IMU_lastUpdateTime) / 2000000.0f);
	}
	halftime = halfT;
	IMU_lastUpdateTime = nowtime;	//更新时间
	//计算重力加速度
	temp = sqrt(ax*ax + ay*ay + az*az);
	temp = (temp / 16384.0f) * 9.8f;   //转成M/S^2为单位的 
	acc_vector = acc_vector +   //低通滤波。截止频率20hz
				(halfT*2.0f / (7.9577e-3f + halfT*2.0f)) * (temp - acc_vector);
	//ax = ax/sqrt(ax^2 + ay^2 + az^2)
	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	//mx = mx/sqrt(mx^2 + my^2 + mz^2)
	norm = invSqrt(mx*mx + my*my + mz*mz);          
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	// compute reference direction of flux
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;     
	
	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
	exInt = exInt + ex * Ki * halfT;
	eyInt = eyInt + ey * Ki * halfT;	
	ezInt = ezInt + ez * Ki * halfT;

	// adjusted gyroscope measurements
	gx = gx + (Kp*ex + exInt);
	gy = gy + (Kp*ey + eyInt);
	gz = gz + (Kp*ez + ezInt);

	}

	// integrate quaternion rate and normalise
	temp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	temp1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	temp2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	temp3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
	q0 = temp0 * norm;
	q1 = temp1 * norm;
	q2 = temp2 * norm;
	q3 = temp3 * norm;
}


#define twoKpDef  (1.0f ) // 2 * proportional gain
#define twoKiDef  (0.2f) // 2 * integral gain

void FreeIMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
	float norm;
//  float hx, hy, hz, bx, bz;
	float vx, vy, vz;
	float ex, ey, ez;
	float temp0,temp1,temp2,temp3;

	// 先把这些用得到的值算好
	float q0q0 = qa0*qa0;
	float q0q1 = qa0*qa1;
	float q0q2 = qa0*qa2;
	float q0q3 = qa0*qa3;
	float q1q1 = qa1*qa1;
	float q1q2 = qa1*qa2;
	float q1q3 = qa1*qa3;
	float q2q2 = qa2*qa2;   
	float q2q3 = qa2*qa3;
	float q3q3 = qa3*qa3;          

	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;   
	
	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3; 
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){

	integralFBx +=  ex * twoKiDef * halftime;
	integralFBy +=  ey * twoKiDef * halftime;	
	integralFBz +=  ez * twoKiDef * halftime;

	gx = gx + twoKpDef*ex + integralFBx;
	gy = gy + twoKpDef*ey + integralFBy;
	gz = gz + twoKpDef*ez + integralFBz;

	}
	// integrate quaternion rate and normalise
	temp0 = qa0 + (double)(-qa1*gx - qa2*gy - qa3*gz)*halftime;
	temp1 = qa1 + (double)(qa0*gx + qa2*gz - qa3*gy)*halftime;
	temp2 = qa2 + (double)(qa0*gy - qa1*gz + qa3*gx)*halftime;
	temp3 = qa3 + (double)(qa0*gz + qa1*gy - qa2*gx)*halftime;  
	
	// normalise quaternion
	norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
	qa0 = temp0 * norm;
	qa1 = temp1 * norm;
	qa2 = temp2 * norm;
	qa3 = temp3 * norm;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
float mygetqval[9];	//用于存放传感器转换结果的数组
void IMU_getQ(float * q) {

	 IMU_getValues(mygetqval);	 
	 //将陀螺仪的测量值转成弧度每秒
	 //加速度和磁力计保持 ADC值　不需要转换
	 IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
									mygetqval[0], mygetqval[1], mygetqval[2],
									mygetqval[6], mygetqval[7], mygetqval[8]);

	 FreeIMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
											mygetqval[0], mygetqval[1], mygetqval[2]);

	q[0] = qa0; //返回当前值	FreeIMU_AHRSupdate 计算出来的四元数 被用到
	q[1] = qa1;
	q[2] = qa2;
	q[3] = qa3;
}

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0f;
	}
	if (v >= 1.0f) {
		return M_PI/2;
	}
	if (v <= -1.0f) {
		return -M_PI/2;
	}
	return asin(v);
}


/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
	static float q[4]; //　四元数
	IMU_getQ(q); //更新全局四元数

	angles[2] = atan2(2.0f*(q[0]*q[1] + q[2]*q[3]),1 - 2.0f*(q[1]*q[1] + q[2]*q[2])) * 180/M_PI;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	angles[1] = -safe_asin(2.0f*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;

	IMU_Yaw = angles[0] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/M_PI; // yaw

//	if(IMU_Yaw <0)IMU_Yaw +=360.0f;  //将 -+180度  转成0-360度

//由于电路板安装时的问题，可能导致组装好飞行器后mpu6050不是水平的，所以用这两个偏置来抵消
  angles[2]+=Quad_Offset_Roll;
	IMU_Roll = angles[2];
  angles[1]+=Quad_Offset_Pitch;
	IMU_Pitch =angles[1];
}

/**************************实现函数********************************************
*函数原型:	   Get_Motion_Acc(void)
*功　　能:  计算运动加速度
输入参数：没有
输出参数：没有
*******************************************************************************/
void Get_Motion_Acc(void)
{
//update20170312加速度计倾角补偿
	float q[4];	
	float norm;
	float temp;
	float Motion_ACC_dt = 0;
	uint32_t nowTime;
	nowTime = micros();  //读取时间

	if (nowTime < Acc_lastUpdateTime)
	{ //定时器溢出过了。
	    Motion_ACC_dt = ((float)(nowTime + (0xffffffff - Acc_lastUpdateTime)) / 1000000.0f);
	    Acc_lastUpdateTime = nowTime;
	    return;
	}
	else
	{
	    Motion_ACC_dt = ((float)(nowTime - Acc_lastUpdateTime) / 1000000.0f);
	}
	Acc_lastUpdateTime = nowTime;

	q[0] = qa0;
	q[1] = qa1;
	q[2] = qa2;
	q[3] = qa3;
	norm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] = q[0] * norm;
	q[1] = q[1] * norm;
	q[2] = q[2] * norm;
	q[3] = q[3] * norm;
	temp = ((float)lastAx * (2.0f * (q[1] * q[3] - q[0] * q[2]))
				 + (float)lastAy * (2.0f * (q[0] * q[1] + q[2] * q[3]))
				 + (float)lastAz * (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
				//  / 16384.0f * 9.8f - Config.ACC_z_zero + 0.04f;//xiang:这个0.0337是我统计出来的在静止时acc_vector与Motion_Accz的差
				 / 16384.0f * 9.8f - Config.ACC_z_zero;
	Motion_Accz = Acc_GetAvg(Motion_Acc_buffer_Z);
	Motion_Accz = Motion_Accz +   //低通滤波。截止频率20hz
				(Motion_ACC_dt / (7.9577e-3f + Motion_ACC_dt)) * (temp - Motion_Accz);
	Motion_Acc_NewValue(Motion_Accx, Motion_Accy, Motion_Accz);
//水平方向用学长的代码，垂直方向用自己的代码
	// float q[4];	
	// float norm;
	// float temp;
	// float Motion_Acc_dt = 0;
	// uint32_t nowTime;
	// float roll_differ, pitch_differ;
	// int direction_roll, direction_pitch;
	// float acc[3];
	// float Acc_Pitch,Acc_Roll;

	// nowTime = micros();  //读取时间
	// if (nowTime < Acc_lastUpdateTime)
	// { //定时器溢出过了。
	//     Motion_Acc_dt = ((float)(nowTime + (0xffffffff - Acc_lastUpdateTime)) / 1000000.0f);
	//     Acc_lastUpdateTime = nowTime;
	//     return;
	// }
	// else
	// {
	//     Motion_Acc_dt = ((float)(nowTime - Acc_lastUpdateTime) / 1000000.0f);
	// }
	// Acc_lastUpdateTime = nowTime;

	// // Acc_NewValue(lastAx,lastAy,lastAz);
	// // acc[0] = Acc_GetAvg(Acc_buffer_X);
	// // acc[1] = Acc_GetAvg(Acc_buffer_Y);
	// // acc[2] = Acc_GetAvg(Acc_buffer_Z);
	// acc[0]=lastAx;
	// acc[1]=lastAy;
	// acc[2]=lastAz;

	// q[0] = qa0;
	// q[1] = qa1;
	// q[2] = qa2;
	// q[3] = qa3;
	// norm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	// q[0] = q[0] * norm;
	// q[1] = q[1] * norm;
	// q[2] = q[2] * norm;
	// q[3] = q[3] * norm;
	// temp = ((float)lastAx * (2.0f * (q[1] * q[3] - q[0] * q[2]))
	// 			 + (float)lastAy * (2.0f * (q[0] * q[1] + q[2] * q[3]))
	// 			 + (float)lastAz * (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
	// 			//  / 16384.0f * 9.8f - Config.ACC_z_zero + 0.04f;//xiang:这个0.0337是我统计出来的在静止时acc_vector与Motion_Accz的差
	// 			 / 16384.0f * 9.8f - Config.ACC_z_zero;//xiang:这个0.0337是我统计出来的在静止时acc_vector与Motion_Accz的差
	// Motion_Accz = Motion_Accz +   //低通滤波。截止频率20hz
	// 			(Motion_Acc_dt / (7.9577e-3f + Motion_Acc_dt)) * (temp - Motion_Accz);

	// //用加速度计算roll、pitch
	// temp = acc[0] * invSqrt((acc[1] * acc[1] + acc[2] * acc[2]));
	// Acc_Pitch = atan(temp);
	// temp = acc[1] * invSqrt((acc[0] * acc[0] + acc[2] * acc[2]));
	// Acc_Roll = atan(temp);

	// roll_differ = IMU_Roll * M_PI / 180 - Acc_Roll;
	// pitch_differ = IMU_Pitch * M_PI / 180 - Acc_Pitch;

	// if (roll_differ < 0.0f)
	//     direction_roll = -1;
	// else
	//     direction_roll = 1;
	// if (pitch_differ < 0.0f)
	//     direction_pitch = -1;
	// else
	//     direction_pitch = 1;

	// roll_differ = tan(roll_differ);
	// pitch_differ = tan(pitch_differ);
	// roll_differ = roll_differ * roll_differ;
	// pitch_differ = pitch_differ * pitch_differ;
	// acc[2] = acc[2] * acc[2];
	// temp = (1 - roll_differ * pitch_differ);

	// if (direction_pitch < 0)
	//     Motion_Accx = (-sqrt(((roll_differ + 1) * pitch_differ * acc[2]) / temp) / 16384 * 9.8f);
	// else
	//     Motion_Accx = (sqrt(((roll_differ + 1) * pitch_differ * acc[2]) / temp) / 16384 * 9.8f);
	// if (direction_roll < 0)
	//     Motion_Accy = (-sqrt(((pitch_differ + 1) * roll_differ * acc[2]) / temp) / 16384 * 9.8f);
	// else
	//     Motion_Accy = (sqrt(((pitch_differ + 1) * roll_differ * acc[2]) / temp) / 16384 * 9.8f);

	// Motion_Acc_NewValue(Motion_Accx, Motion_Accy, Motion_Accz);
	// Motion_Accx = Acc_GetAvg(Motion_Acc_buffer_X);
	// Motion_Accy = Acc_GetAvg(Motion_Acc_buffer_Y);
	// Motion_Accz = Acc_GetAvg(Motion_Acc_buffer_Z);
}

/**************************实现函数********************************************
*函数原型:	   Get_Motion_Velocity(void)
*功　　能:  计算运动速度
输入参数：没有
输出参数：没有
*******************************************************************************/
void Get_Motion_Velocity()
{
//update20170110
	static float Motion_Accx_last = 0, Motion_Accy_last = 0, Motion_Accz_last = 0;
	uint32_t nowTime;
	nowTime = micros();  //读取时间
	Get_Motion_Acc();

	if (nowTime < Velocity_lastUpdateTime)
	{ //定时器溢出过了。
	    Motion_Velocity_dt = ((float)(nowTime + (0xffffffff - Velocity_lastUpdateTime)) / 1000000.0f);
	    Velocity_lastUpdateTime = nowTime;
	    return;
	}
	else
	{
	    Motion_Velocity_dt = ((float)(nowTime - Velocity_lastUpdateTime) / 1000000.0f);
	}
	Velocity_lastUpdateTime = nowTime;

	//对运动加速度做积分求运动速度
	Motion_Velocity_X += ((Motion_Accx + Motion_Accx_last) * Motion_Velocity_dt) / 2.0f;
	Motion_Velocity_Y += ((Motion_Accy + Motion_Accy_last) * Motion_Velocity_dt) / 2.0f;
	Motion_Velocity_Z += ((Motion_Accz + Motion_Accz_last) * Motion_Velocity_dt) / 2.0f;
	Motion_Accx_last = Motion_Accx;
	Motion_Accy_last = Motion_Accy;
	Motion_Accz_last = Motion_Accz;
}

 /**************************实现函数********************************************
*函数原型:	   void Get_Position(void)
*功　　能:  使用加速度计算位置。
输入参数：没有
输出参数：没有
*******************************************************************************/
void Get_Position()
{
    //update20170110
	uint32_t nowTime;
    static float Velocity_x_last = 0, Velocity_y_last = 0, Velocity_z_last = 0;
	nowTime = micros();  //读取时间
    Get_Motion_Velocity();
    if (nowTime < Position_lastUpdateTime)
    { //定时器溢出过了。
		Motion_Position_dt = ((float)(nowTime + (0xffffffff - Position_lastUpdateTime)) / 1000000.0f);
		Position_lastUpdateTime = nowTime;
		return;
    }
    else
    {
		Motion_Position_dt = ((float)(nowTime - Position_lastUpdateTime) / 1000000.0f);
    }
    Position_lastUpdateTime = nowTime;

	Position_X += ((Motion_Velocity_X + Velocity_x_last) * Motion_Position_dt) / 2.0f;
	Position_Y += ((Motion_Velocity_Y + Velocity_y_last) * Motion_Position_dt) / 2.0f;
    Position_Z += ((Motion_Velocity_Z + Velocity_z_last) * Motion_Position_dt) / 2.0f;

	Velocity_x_last = Motion_Velocity_X;
	Velocity_y_last = Motion_Velocity_Y;
    Velocity_z_last = Motion_Velocity_Z;
}

//------------------End of File----------------------------
