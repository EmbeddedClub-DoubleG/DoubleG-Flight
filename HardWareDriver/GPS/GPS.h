#ifndef __GPS_H
#define __GPS_H

#include "stm32f4xx.h"


#define _X  1
#define _Y  0

// GPS 模块对外引出的API 程序
extern void Initial_UART3(u32 baudrate);  //初始化
extern void GPS_Routing(void);	//GPS 线程，需要在主循环中不断调用
extern void GPS_Reset_Home(void); //重新记录家点，将当前位置定义为家点
extern float GPS_Distance(float lat1,float lon1,float lat2,float lon2);
extern float GPS_Heading(float lat1,float lon1,float lat2,float lon2);


//------------以下数据由  void GPS_Routing(void); 子程序进行更新--------
extern unsigned char GPS_STA_Num , //使用卫星数量，从00到12
			  GPS_Update , //GPS 数据更新提示  1=数据已更新  由外部清零
			  GPS_Locked ; //定位状态，1=有效定位，0=无效定位
extern double GPS_Altitude , //天线离海平面的高度，-9999.9到9999.9米
	  Latitude_GPS , //纬度	 单位为度
	  Longitude_GPS , //经度  单位为度
	  Speed_GPS , //地面速率  单位  米每秒
	  Course_GPS ; //地面航向（000.0~359.9度，以真北为参考基准)
extern volatile unsigned char GPS_Time[8]; //UTC 时间，hhmmss（时分秒）格式
extern volatile unsigned char GPS_Date[8]; //UTC日期，ddmmyy（日月年）格式
extern unsigned char Home_Ready ; //家点提取标志
//--------家点的信息---------
extern float Home_Latitude , Home_Longitude , Home_Altitude 
				,GPS_Period;  //两次GPS更新的时间差，也就是GPS的更新间隔 单位 秒
extern float now_speed[2];

#endif

//------------------End of File----------------------------
