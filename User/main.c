#include "common.h"//这是所有驱动程序的总合，用户只需要添加这个文件就可以操作所有的硬件。
#include "FAT_driver.h"//FatFs是一个通用的文件系统模块
#include <stdio.h>
#include "Camera.h"
//数据上传上位机的状态机
#define REIMU  0x01 //上传解算的姿态数据
#define REMOV  0x02	//上传传感器的输出
#define REHMC  0x03	//上传磁力计的标定值。xiang：这个东西没用到

#define Upload_Speed  10   //数据上传速度  单位 Hz;
#define upload_time (1000000/Upload_Speed)/2	//单位us。xiang：上传时间间隔。这个不是真的上传周期，(1000000/Upload_Speed)才是上传周期。
												//除以2的原因是一次上传只上传一部分数据，具体除以多少要看全部的数据是分多少次上传的。

//int16_t ax, ay, az;	//accel是加速度计
//int16_t gx, gy, gz;//gyro是陀螺仪
//int16_t hx, hy, hz;//hmc是磁力计
uint32_t system_micrsecond;//记录上一次发送数据给上位机时的系统时间
float ypr[3]; // yaw pitch roll，yaw是航向角 pitch是俯仰角 rool是横滚角
u8  Mon_PID_CH = 2; //控制当前向上位机发送ROLL PID数据还是PITCH PID、YAW  PID、高度 PID。默认监控PITCH PID。由上位机命令设置xiang:这里本来默认是ROLL，我改成了PITCH
int16_t Math_hz=0;//xiang：记录从上一次上传姿态解算的次数信息给上位机到这次上传期间传姿态解算的次数。Math_hz*10是每秒姿态解算的次数，因为上传姿态解算的频率是10hz
//update20170113
double systemTime = 0;
uint32_t systemTimeLast = 0;

#if Yingzhang_GCS
void GCS_Upload(void);
void GCS_GetCommand(unsigned char PC_comm);	
#endif

#if Captain_GCS
void UART1_Monitor_PID(uint8_t index);
void GCS_Upload(void);
void GCS_GetCommand(unsigned char PC_comm);	
#endif

void Remote_Command(void);
/**************************实现函数********************************************
*函数原型:		int main(void)
*功　　能:		主程序
*******************************************************************************/
int main(void)
{
#if Captain_GCS|Yingzhang_GCS
	unsigned char PC_comm; //PC发送的命令
#endif
	uint32_t systemTimeNow;//update20170113
	/* 配置系统时钟为120M 使用外部8M晶体+PLL*/      
	// SystemInit();//系统会自动调用的，所以这里不需要手动调用
	NVIC_PriorityGroupConfig(NVIC_GROUPS);//中断优先级分组	定义于fly_config.h
	delay_init(168);//延时初始化  并启动开机时间。
	Initial_System_Timer();//启动系统主定时器
	systemTime += ((double)(micros() - systemTimeLast)) / 1000000.0;//update20170113
	Initial_LED_GPIO();//初始化板子上的LED接口
	LEDRed_ON();
	AT45db161_SPI_Configuration();//初始化 SPI总线
	SPI3_Configuration();
	ADC_Voltage_initial();//ADC初始化
	Initial_UART1(115200L);//主通信接口,uart1用于和上位机通信
	// Initial_UART3(GPS_Baudrate); //GPS 接口设置	定义于fly_config.h,uart3用于和GPS通信
	Initial_UART4(115200);

	IIC_Init();	 //初始化I2C接口 
	PWM_Input_Initial(); 
	PWM_Output_Initial();
	delay_ms(300);//等待器件上电
	IMU_init(); //初始化IMU和传感器	
	delay_ms(20);
	// FAT_Initial();//SD卡初始化
	// mf_open("/test2.txt",FA_READ|FA_WRITE|FA_CREATE_ALWAYS);
	// mf_read(50);
	// mf_write(" i love you.",12);
	// mf_close();
	while(AT45DB_Check()==0)
	{//查询AT45DB 是否在电路板上
		LEDRed_ON();	 //如果没有，则程序死在这里
		delay_ms(100);	//红灯闪烁
		LEDRed_OFF();
		delay_ms(100);
	}
	if(PWM_Input_CH3 > 1500)
	{//带油门上电， 说明用户希望 初始化电调
		PWM_Output_ESC_Calibration(); //电调行程设置函数，参考readme和电调说明书
		Quadrotor_Mode = Quad_ESC_Cal;//四轴状态 = 正在校准ESC
	}

	Ultrasonic_initial(); //初始化 超声波测距仪
	Initial_Quadrotor_Math(); //初始化 四轴姿态PID参数

	LEDRed_OFF();  //关灯
	system_micrsecond=micros();	//读取系统时间，开始计时
	while(1)//主循环
	{
		// update20170113
		systemTimeNow = micros();
		if (systemTimeNow <= systemTimeLast)
			systemTime += ((double)(systemTimeNow + (0xffffffff - systemTimeLast))) / 1000000.0f;//systemTime单位是s
		else
			systemTime += ((double)(systemTimeNow - systemTimeLast)) / 1000000.0f;
		systemTimeLast = systemTimeNow;
		
		// if(cameraReady!=1 && systemTime>2.0)
		// {
		//     Camera_Int();
		// }

		//基础线程服务 用于解算当前姿态，更新各种数据
		IMU_getYawPitchRoll(ypr); //姿态更新
		FLOW_getData();//光流采集数据更新
		// Get_Position();//update20170110
		Math_hz++; //解算次数 ++
		Ultrasonic_Routine(); // 超声波测距线程
		MS5611BA_Routing(); //处理MS5611 事务，ms5611是气压计，用来测量高度的
		LED_Blink_Routine(); // LED闪烁线程
		//GPS_Routing();	    // GPS数据处理线程
		Remote_Command();  // 查看一下，遥控器有没有什么命令过来
		
		//更新PWM输出，以控制四轴
		if(Servo_Update_Req)//这个标志位控制PWM输出周期，由定时器中断置1,见PWM_Output.c
		{//PWM输出更新
			Quadrotor_Motor_Update(); //计算四个电机的转速 并输出 
			Servo_Update_Req = 0; //清标志位 等待一下次的更新
		}

#if Captain_GCS||Yingzhang_GCS
		if((micros()-system_micrsecond)>upload_time)//是否到了更新 上位机的时间了
			GCS_Upload();//将姿态数据传给上位机
		if((PC_comm=UART1_CommandRoute())!=0xff)
			GCS_GetCommand(PC_comm);// 处理PC 发送的命令
#endif

#if SerialDebug
		//上传数据给串口调试助手
		if((micros()-system_micrsecond)>upload_time)//单位us
		{
			char string_to_send1[80]={0};
			char string_to_send2[80]={0};

			sprintf(string_to_send1, "\r\nPitch:%f Target:%f Y_Speed:%d Ymove:%f \r\n",IMU_Pitch,-Target_Pitch,Y_Speed,Ymove);
			 UART1_Put_String((unsigned char *)string_to_send1);
			sprintf(string_to_send2, "\r\nRoll:%f Target:%f X_Speed:%d Xmove:%f \r\n",IMU_Roll,Target_Roll,X_Speed,Xmove);
			 UART1_Put_String((unsigned char *)string_to_send2);

			system_micrsecond=micros();
		}
#endif
	}//主循环 while(1) 结束
}//main	

#if Captain_GCS
/**************************实现函数********************************************
*函数原型:		void UART1_Monitor_PID(uint8_t index)
*功　　能:		将当前PID的目标值，当前值，控制量 上传到PC 监控
*******************************************************************************/
void UART1_Monitor_PID(uint8_t index){
	if(Quadrotor_Mode == Quad_Manual)
		return;

	switch(index){
	case 1:
	UART1_Report_PID(Stabilize_Roll.target, // ROLL
						Stabilize_Roll.current,
						//Stabilize_Roll.PID_out);
						RollRate.PID_out);
	case 2:
	UART1_Report_PID(Stabilize_Pitch.target, // pitch
						Stabilize_Pitch.current,
						PitchRate.PID_out);
						break ;
	case 3:
	UART1_Report_PID(Stabilize_Yaw.target, // yaw
						Stabilize_Yaw.current,
						YawRate.PID_out);
						break ;
	case 4:
	UART1_Report_PID(Climb.target, // 高度
						Climb.current,
						Climb.PID_out);
						break ;
	}
}
#endif

#if Captain_GCS
/**************************实现函数********************************************
*函数原型:		void GCS_Upload(void)
*功　　能:		将姿态数据传给上位机
*******************************************************************************/
void GCS_Upload(void)
{
	static u8 count_temp = 0;//每上传数据给上位机五次有一次上传系统状态数据帧
	static u8 state = REIMU;//数据上传上位机状态
	int32_t Temperature = 0, Pressure = 0, Altitude = 0;//发送给上位机的温度气压高度数据
	switch(state)
	{
		case REIMU://发送解算后的姿态
		Temperature = MS5611_Temperature/10; //读取最近的温度值
		Pressure = MS5611_Pressure;	   //读取最近的气压测量值
		// Altitude = GetAltitude()/10.0f;	   //读取相对高度xiang:这里本来是用气压计的高度的//update20161227
		Altitude = Filter_Altitude/10.0f;	   //读取相对高度xiang:这里本来是用气压计的高度的
		UART1_ReportIMU((int16_t)(ypr[0]*10.0f),(int16_t)(ypr[1]*10.0f),(int16_t)(ypr[2]*10.0f),//发送数据
			Altitude,Temperature,Pressure/10,Math_hz*10);
		UART1_Monitor_PID(Mon_PID_CH); //监控PID
		Math_hz=0;
		state = REMOV; //更改状态。
		UART1_Report_PWMout(); //发送PWM输出 到上位机
		break;
		
		case REMOV://发送传感器数据
		UART1_ReportMotion(lastAx,lastAy,lastAz,lastGx,lastGy,lastGz,lastMx,lastMy,lastMz);
		// UART1_ReportMotion(200,200,200,200,200,200,200,200,200);
		UART1_Report_PWMin();//发送PWM输入值
		state = REIMU;
			
		break;
		default: 
		state = REIMU;
		break;
	}//switch(state)
	system_micrsecond=micros();	 //取系统时间 单位 us 
	//LED_Set_Blink(Blue,60,100,1);

	if(count_temp++ > 5)
	{
		count_temp = 0;
		UART1_ReportSysteminfo(	  //发送系统状态数据帧
							Quadrotor_Mode,	 // 飞行模式
							Get_Bat_Vol(),	 // 主电池电压 
							Get_Sevor_Vol(), // 舵机电压
							0,				 // 错误代码
							I2C_Erorr_Count,  //I2C 总线错误次数
							system_micrsecond/1000000  //系统时间
							);
		if(Mag_calib)UART1_ReportHMC(Mag_maxx,Mag_maxy,Mag_maxz,
			 Mag_minx,Mag_miny,Mag_minz,0);//发送磁力计标定值
	}
}
#elif Yingzhang_GCS
/**************************实现函数********************************************
xiang：这个函数是我自己写的，为了用自己的上位机，用于替代captain原来的函数
*函数原型:		void GCS_Upload(void)
*功　　能:		将姿态数据传给上位机
*******************************************************************************/
void GCS_Upload(void)//xiang：注意：这个函数是针对自己写的上位机的，如果要用captain上位机，就用上面那个函数
{
//	switch(Mon_Data)
//	{
//		case Mon_Height:
//			UART1_ReportHeight((float)Filter_Altitude/100.0f);
//			Mon_Data=Mon_No;
//			break;
//	}	
}
#endif

#if Captain_GCS
/**************************实现函数********************************************
	*函数原型:		void GCS_GetCommand(unsigned char PC_comm)
	*功　　能:		处理PC发送来的命令
	*输入：PC_comm：pc发送来的命令标识
*******************************************************************************/
void GCS_GetCommand(unsigned char PC_comm)
{
	//LED_Set_Blink(Red,100,100,4);  // 红色的LED 闪烁表示正在处理 PC发送的命令
	switch(PC_comm)//检查命令标识
	{
		case Gyro_init://读取陀螺仪零偏
									MPU6500_InitGyro_Offset(); 
									Config.ACC_z_zero = acc_vector;//记录此时的重力值。
									AT45DB_Write_config();
									break;
									
		case High_init:		MS561101BA_ResetAlt();		break;//气压高度 清零
		
		case HMC_calib_begin:	LSM303_Start_Calib();	break;     //启动磁力计标定
		case HMC_calib:		    LSM303_Save_Calib();	break;     //保存磁力计标定

		case 0xB0:	   UART1_Return_Setting();			break;
		case 0xB1:     UART1_Get_Setting();					break;

		case 0xB2:
							UART1_Return_PID(&Stabilize_Roll,&RollRate,0);
							UART1_Return_PID(&Stabilize_Pitch,&PitchRate,1);
							UART1_Return_PID(&Stabilize_Yaw,&YawRate,2);
							UART1_Return_PID(&AutoHigh_THR,&Climb,3);
							UART1_Return_PID(&Position_Hold,&Position_Speed,4);
							break;

		//PC机发送用户修改的PID参数值，提取PID参数值，并将参数写到AT45DB中掉电保存
		case 0xB3:     UART1_Get_PID(&Stabilize_Roll,&RollRate);  break;
		case 0xB4:     UART1_Get_PID(&Stabilize_Pitch,&PitchRate);  break;
		case 0xB5:     UART1_Get_PID(&Stabilize_Yaw,&YawRate);  break;
		case 0xB6:     UART1_Get_PID(&AutoHigh_THR,&Climb);  break;
		case 0xB7:     UART1_Get_PID(&Position_Hold,&Position_Speed);  break;

		//上位机监控PID
		case 0xd6:  Mon_PID_CH  = 1; break; //监控 ROLL	PID
		case 0xd7:  Mon_PID_CH  = 2; break;	//监控 PITCH PID
		case 0xd8:  Mon_PID_CH  = 3; break;	//监控 YAW  PID
		case 0xd9:  Mon_PID_CH  = 4; break; //监控 高度 PID

		case 0xf5:  PWM_Save_Offset(); break; //PWM零点记录
	}
}
#elif Yingzhang_GCS
/**************************实现函数********************************************
	xiang：这个函数是我自己写的，为了用自己的上位机，用于替代captain原来的函数
	*函数原型:		void GCS_GetCommand(unsigned char PC_comm)
	*功　　能:		处理PC发送来的命令
	*输入：PC_comm：pc发送来的命令标识
*******************************************************************************/
void GCS_GetCommand(unsigned char PC_comm)//xiang：注意：这个函数是针对自己写的上位机的，如果要用captain上位机，就用上面那个函数
{
	//LED_Set_Blink(Red,100,100,4);  // 红色的LED 闪烁表示正在处理 PC发送的命令
	switch(PC_comm)//检查命令标识
	{
		case 0x10:			UART1_ReportHeight(Filter_Altitude);		break; //监控高度 单位cm
		case 0x11:		    UART1_ReportTHR();		    break;
		case 0x51://发送当前姿态角以及9轴传感器数据到营长GCS
		   			UART1_ReportIMUMotion(ypr[0], ypr[1], ypr[2],
					  						lastAx, lastAy, lastAz, lastGx, lastGy, lastGz, lastMx, lastMy, lastMz,
					  						Filter_Altitude/100.0f, MS5611_Temperature / 100.0f, MS5611_Pressure);
		    		break;
		case 0x61:		    UART1_Report_PWMInOut();		    break;//将PWM输入和输出的脉宽值发送到PC
		//上位机监控PID当前值和目标值
		case 0x21:		    UART1_Monitor_AutoHigh();		    break;

		//上位机读取PID参数
		case 0x31:			UART1_Return_PID(&AutoHigh_THR,&Climb);			break;
		case 0x32:			UART1_Return_PID(&Stabilize_Roll,&RollRate);			break;
		case 0x33:			UART1_Return_PID(&Stabilize_Pitch,&PitchRate);			break;
		case 0x34:			UART1_Return_PID(&Stabilize_Yaw,&YawRate);			break;
		case 0x35:			UART1_Return_PID(&Position_Hold,&Position_Speed);			break;

		//PC机发送用户修改的PID参数值，提取PID参数值，并将参数写到AT45DB中掉电保存
		case 0x41:		    UART1_Get_PID(&AutoHigh_THR, &Climb);		    break;
		case 0x42:		    UART1_Get_PID(&Stabilize_Roll, &RollRate);		    break;
		case 0x43:		    UART1_Get_PID(&Stabilize_Pitch, &PitchRate);		    break;
		case 0x44:		    UART1_Get_PID(&Stabilize_Yaw, &YawRate);		    break;
		case 0x45:		    UART1_Get_PID(&Position_Hold, &Position_Speed);		    break;

		case Gyro_init: //读取陀螺仪零偏
		    MPU6500_InitGyro_Offset();
		    Config.ACC_z_zero = acc_vector; //记录此时的重力值。
		    AT45DB_Write_config();
		    break;

		case High_init:		MS561101BA_ResetAlt();		break;//气压高度 清零
		
		case HMC_calib_begin:	LSM303_Start_Calib();	break;     //启动磁力计标定
		case HMC_calib:		    LSM303_Save_Calib();	break;     //保存磁力计标定
		case 0x71:
		    UART1_ReportHMC(Mag_maxx, Mag_maxy, Mag_maxz,
				    Mag_minx, Mag_miny, Mag_minz); //发送磁力计标定值
		    break;
		//读取上位机控制命令
		//ch1增加往右，ch2增加往前，ch4增加往右
		case 0xc1:		    GCSControl_CH2 = 75;			break;//前进
		case 0xc2:		    GCSControl_CH2 = -75;		    break;//后退
		case 0xc3:		    GCSControl_CH1 = -75;		    break;//左倾
		case 0xc4:		    GCSControl_CH1 = 75;		    break;//右倾
		case 0xc5:		    GCSControl_CH4 = -100;		    break;//左转
		case 0xc6:		    GCSControl_CH4 = 100;		    break;//右转
		case 0xc7:
		    GCSControl_CH1 = 0;
		    GCSControl_CH2 = 0;
		    GCSControl_CH3_Accumulate = 0;
		    GCSControl_CH4 = 0;
		    break;
		case 0xc8:		    GCSControl_CH3_Accumulate = 0.1;		    break; //加油门
		case 0xc9:		    GCSControl_CH3_Accumulate = -0.1;		    break; //减油门

		case 0x81:		    Camera_Routine();		    break;   //发摄像头采集图像
		case 0xd1:		    Quadrotor_Mode = Quad_Take_Of;		    break;
		case 0xd2:		    Quadrotor_Mode = Quad_Landing;		    break;
	}
}
#endif
/*
//当油门小于1/10且方向打到最右与转向打到最右则校准陀螺仪
//当油门小于1/10且方向打到最右与仰俯打到最上启动磁力计标定
//当油门小于1/10且方向打到最右与仰俯打到最下保存磁力计标定
//当油门小于1/10且方向打到最左与转向打到最左则锁定油门
电调控制无刷电机发出无PWM信号，每隔两秒BI一声
//当油门小于1/10且方向打到最左与转向打到最右则解锁油门
电调控制无刷响一声长声（每次节电第一次解锁）
//每次接电启动飞控将会自动锁定PWM输出为0
需要遥控器解锁才可继续操作
带油门上电会自动解锁进入校准油门行程
修改了之前YAW角度换算问题
加入了速度计信任度计算
*/
void Remote_Command(void)
{
#if (Remote_Config_Enable == 1)	 //条件编译，Remote_Config_Enable的定义见fly_config.h

	if(PWM_Input_CH3<(int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10)&&PWM_Input_CH4 > 1800  && PWM_Input_CH1 > 1800) //当油门小于1/10且方向打到最右与转向打到最右则校准陀螺仪
		MPU6500_InitGyro_Offset();			//陀螺仪水平标定 需要在四轴静止的时候做
	if(PWM_Input_CH3<(int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10)&&PWM_Input_CH4 > 1800  && PWM_Input_CH2 > 1800)	//当油门小于1/10且方向打到最右与仰俯打到最上启动磁力计标定
		LSM303_Start_Calib();     //启动磁力计标定  启动后，按规定转几圈，参考上位机的方法
	if(PWM_Input_CH3<(int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10)&&PWM_Input_CH4 > 1800  && PWM_Input_CH2 < 1400)	//当油门小于1/10且方向打到最右与仰俯打到最下保存磁力计标定
		LSM303_Save_Calib();     //保存磁力计标定
	if(PWM_Input_CH3<(int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10)&&PWM_Input_CH4 < 1400  && PWM_Input_CH1 < 1400) //当油门小于1/10且方向打到最左与转向打到最左则锁定油门
	{
		THROTTLE_LOCKed = 1;	//锁定标志位  该变量将会在 PWMoutput.c中用到
	}		
	if(PWM_Input_CH3<(int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10)&&PWM_Input_CH4 < 1400  && PWM_Input_CH1 > 1800) //当油门小于1/10且方向打到最左与转向打到最右则解锁油门
	{
		THROTTLE_LOCKed = 0;//解锁标志位  这样，你就可以控制四轴了。
	}

#endif
}

//------------------End of File----------------------------
