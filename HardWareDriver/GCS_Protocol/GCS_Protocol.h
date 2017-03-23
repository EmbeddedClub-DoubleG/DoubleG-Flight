#ifndef __GCS_PRO_H
#define __GCS_PRO_H

#include "stm32f4xx.h"
#include "Quadrotor.h"
#include "GPS.h"
#include "UART1.h"
#include "Fmath.h"
#include "fly_config.h"
#define Gyro_init  0xE0
#define High_init  0xE2
#define HMC_calib  0xE1
#define HMC_calib_begin  0xE3

#if Captain_GCS
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,
					int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
void UART1_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz,
					int16_t minx,int16_t miny,int16_t minz,int16_t IMUpersec);
void UART1_Report_PWMin(void);
void UART1_Report_PWMout(void);
void UART1_Report_PID(float target,float current,float pidcon);
void UART1_Return_PID(struct Quad_PID *PID1,struct Quad_PID *PID2,unsigned char index);//将PID参数返回给上位机	  将两组关联的PID发送给上位机
void UART1_Return_Setting(void);//响应上位机读取设置项
#elif Yingzhang_GCS
void UART1_ReportHeight(float height);
void UART1_ReportTHR(void);
void UART1_Monitor_AutoHigh(void);
void UART1_Return_PID(struct Quad_PID *PID1, struct Quad_PID *PID2);
void UART1_ReportIMUMotion(float yaw, float pitch, float roll,
		     int16_t ax, int16_t ay, int16_t az,
		     int16_t gx, int16_t gy, int16_t gz,
		     int16_t hx, int16_t hy, int16_t hz,
		     float alt, float tempr, float press);
void UART1_ReportHMC(int16_t maxx, int16_t maxy, int16_t maxz, int16_t minx, int16_t miny, int16_t minz);
void UART1_Report_PWMInOut(void);
#endif

extern void UART1_ReportSysteminfo(
						int8_t mode,
						int16_t bat_vol,
						int16_t sevor_vol,
						int8_t error_code,
						int16_t I2C_error_count,
						int16_t	system_time   //ms
						);


extern void UART1_ReportPosition(
						int32_t lon,
						int32_t lat,
						int16_t hight,
						int8_t  STnum,
						int16_t heading,
						int16_t	speed
						);


extern void UART1_ReportTarget(
						int32_t lon,
						int32_t lat,
						int16_t hight,
						int16_t Distance,
						int16_t Xspeed,
						int16_t	Yspeed
						);



#endif

//------------------End of File----------------------------
