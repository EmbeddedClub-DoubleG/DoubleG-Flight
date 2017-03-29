#ifndef __Quadrotor_H
#define __Quadrotor_H

#include "stm32f4xx.h"
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
//定高相关变量
#define Default_Throttle 1470.0f//默认油门

extern volatile uint8_t  Fly_Mode;
extern volatile int16_t  Yaw_DIRECT;
extern int16_t THROTTLE,PID_ROLL,PID_PITCH,PID_YAW;
extern volatile float PID_dt;
extern int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
extern volatile uint8_t Quadrotor_Mode;	//飞行模式
extern float Tartget_hight;
//光流定点相关全局变量
extern float RollAdd,PitchAdd;

/*---------------------接口函数声明------------------------*/
void Quadrotor_Motor_Update(void);
void Initial_Quadrotor_Math(void);
void PWM_Save_Offset(void);
void PWM_Output_ESC_Calibration(void);

#endif
