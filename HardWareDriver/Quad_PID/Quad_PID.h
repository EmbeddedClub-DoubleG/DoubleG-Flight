#ifndef __Quad_PID_H
#define __Quad_PID_H

extern struct Quad_PID 
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

//定高相关全局变量
extern volatile float Increas_Output_Accumulat;

extern float GPS_PITCH,GPS_ROLL;

void Height_PID_Reset(void);
void Roll_Pitch_Yaw_RatePID(float Rate_roll,float Rate_pitch,float Rate_yaw);
void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw);
void Position_Hold_Reset(void);
void GPS_Position_Hold(void);
float Z_Speed_PID(float Speed);
float Height_PID(float height);

#endif
