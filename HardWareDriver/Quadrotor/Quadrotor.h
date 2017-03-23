#ifndef __Quadrotor_H
#define __Quadrotor_H

#include "stm32f4xx.h"
#include "PWM_Input.h"
#include "PWM_Output.h"
#include "IMU.h"
#include "pid.h"
#include "fly_config.h"
#define  CVERSION  1.4


//xiang：飞行器飞行模式
#define Quad_Manual			0x01//xiang：手动模式
#define Quad_Level_Lock		0x02//xiang：平衡模式
#define Quad_Hold_Position	0x03//xiang：定点模式
#define Quad_ESC_Cal		0x04//xiang：电调校准模式：启动时若CH3>1500则校准电调
#define Quad_Assignment		0x05//xiang：这个是我自己添加的任务模式
#define Quad_Landing		0x06//降落模式
#define Quad_Take_Of		0x07//起飞模式
#define Quad_Auto_High		0x08//定高模式
//xiang：这段代码是我加的
#if BoardB_Enable//update20161226:增加BoardB条件编译
//主板B的命令
#define Command_Stay 0x01//保持当前高度和位置（默认）
#define Command_Up 0x02//上升
#define Command_Down 0x03//下降
#define Command_Forward 0x04//前移
#define Command_Back 0x05//后移
#define Command_Left 0x06//左移
#define Command_Right 0x07//右移
//发送给主板B的消息
#define Message_Start 0x01//已经进入任务模式，主板B可以给主板A发送命令了
#endif

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
extern volatile int16_t  Yaw_DIRECT ;
extern int16_t THROTTLE,PID_ROLL,PID_PITCH,PID_YAW;
extern float  Target_Roll,Target_Pitch,Target_Yaw,Tartget_hight;
extern int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
extern volatile uint8_t Quadrotor_Mode;	//飞行模式
// extern int16_t tpitch,troll,tyaw,pidpitch,pidroll,pidyaw;//update20161227:没用到，注释掉
extern volatile float Increas_Output_Accumulat;
extern float Height_PID_Out;
extern float Land_targethigh;
extern float RollAdd,PitchAdd;
extern uint8_t High_Flag_IsLanded;
extern struct Quad_PID 
				Stabilize_Roll,  // 横滚 PID
				Stabilize_Pitch,  // 府仰 PID
				RollRate,		  // 横滚 角速率PID
				PitchRate,		  // 府仰 角速率PID
				Stabilize_Yaw,	  //航向 PID
				YawRate,          //航向 角速率PID
				AutoHigh_THR,	  //定高 PID
				Climb,            //爬升 PID
				Position_Hold,	  //位置 定点PID
				Position_Speed;	  //速度 PID

void Quadrotor_Motor_Update(void);
void Initial_Quadrotor_Math(void);
void PWM_Save_Offset(void);
void PWM_Output_ESC_Calibration(void);
// float GetAltitude(void);//update20161227
// float GetZSpeed(void);
#endif

//------------------End of File----------------------------
