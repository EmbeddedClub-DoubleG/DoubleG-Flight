#include"AltitudeFilter.h"
#include"MS5611.h"
#include"Ultrasonic.h"
#include"IMU.h"

float Filter_Altitude=0.0f;//气压计和超声波和加速度计滤波得到的高度，单位cm
float Filter_Altitude_D = 0;//高度变化率，即垂直速度，单位cm/s
volatile float evaluateAltitude = 0;
float ALT_Update_Interval = 0.0; //两次高度测量，之间的时间间隔
float D2_Alt = 0;//对高度去两次微分

void Get_Filter_Altitude(void)
{
    
}

void Altitude_Get_D(void)
{

}
