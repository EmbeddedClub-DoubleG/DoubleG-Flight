#ifndef __common_H
#define __common_H

/*
这是所有驱动程序的总合，用户只需要添加这个文件就可以操作所有的硬件。
详细的API 请参考各个驱动的头文件。
*/
#include "STM32ADC.h"
#include "pid.h"
#include "GCS_Protocol.h"
#include "GPS.h"
#include "Ultrasonic.h"
#include "fly_config.h"
#include "LED.h" 
#include "Quadrotor.h"
#include "UART1.h"
#include "IOI2C.h"
#include "delay.h"
#include "MS5611.h"
#include "LSM303D.h"
#include "MPU6500.h"
#include "SPI3.h"
#include "IMU.h"
#include "AT45DB.h"
#include "PWM_Input.h"
#include "PWM_Output.h"
#include "datamap.h"
#include "Camera.h"
#include "UART4.h"
#include "flow.h"

extern double systemTime;

//浮点 联合体
typedef union {
	float  value;
	unsigned char byte[4];
} f_bytes;

//整数 联合体
typedef union {
	int16_t  value;
	unsigned char byte[2];
} i_bytes;

// //双精浮点 联合体
// typedef union{
//     double value;
//     unsigned char byte[8];
// } d_bytes;
#endif

//------------------End of File----------------------------



