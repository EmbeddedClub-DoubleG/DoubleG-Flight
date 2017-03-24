#ifndef __Quadrotor_H
#define __Quadrotor_H

#include "stm32f4xx.h"
#include "PWM_Input.h"
#include "PWM_Output.h"
#include "IMU.h"
#include "pid.h"
#include "fly_config.h"
/*---------------------宏定义------------------------*/
//飞行器飞行模式
#define Quad_Manual			0x01//手动模式
#define Quad_Level_Lock		0x02//平衡模式
#define Quad_Hold_Position	0x03//定点模式
#define Quad_ESC_Cal		0x04//电调校准模式：启动时若CH3>1500则校准电调
#define Quad_Assignment		0x05//任务模式
#define Quad_Landing		0x06//降落模式
#define Quad_Take_Of		0x07//起飞模式
#define Quad_Auto_High		0x08//定高模式

/*---------------------全局变量声明定义------------------------*/
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
				
#if Yingzhang_GCS
//update20170113
//上位机控制四轴目标角度相关变量
extern int8_t GCSControl_Forward;
extern int8_t GCSControl_Backward;
extern int8_t GCSControl_Leftward;
extern int8_t GCSControl_Rightward;
extern int8_t GCSControl_LeftRotate;
extern int8_t GCSControl_RightRotate;
#endif

extern volatile uint8_t  Fly_Mode;
extern volatile int16_t  Yaw_DIRECT;
extern int16_t THROTTLE,PID_ROLL,PID_PITCH,PID_YAW;
extern float  Target_Roll,Target_Pitch,Target_Yaw,Tartget_hight;
extern int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
extern volatile uint8_t Quadrotor_Mode;	//飞行模式
//定高相关全局变量
extern volatile float Increas_Output_Accumulat;
extern float Height_PID_Out;
//光流定点相关全局变量
float RollAdd,PitchAdd;

/*---------------------接口函数声明------------------------*/
void Quadrotor_Motor_Update(void);
void Initial_Quadrotor_Math(void);
void PWM_Save_Offset(void);
void PWM_Output_ESC_Calibration(void);

#endif
