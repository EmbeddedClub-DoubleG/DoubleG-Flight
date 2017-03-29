#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx.h"

#define M_PI  (float)3.1415926535

//解算后的姿态输出，供外部程序调用
extern volatile float IMU_Pitch,IMU_Roll,IMU_Yaw;	  //单位 度
//陀螺仪的输出，
extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	//单位  度每秒
extern float acc_vector ;  //当前加速度感应到的力合  M/S^2。
//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * ypr); //更新姿态  需要定期调用
//update20170110
void Get_Position(void);//通过加速度计二次累加计算位移 需要定期调用
extern volatile float Motion_Accx, Motion_Accy, Motion_Accz;
extern volatile float Motion_Velocity_X, Motion_Velocity_Y, Motion_Velocity_Z;
extern volatile float Position_X, Position_Y, Position_Z;

#endif

//------------------End of File----------------------------
