#include"AltitudeFilter.h"
#include"MS5611.h"
#include"Ultrasonic.h"
#include"IMU.h"

#define MOVAVG_SIZE 10
const float  FiltAlt_Lowpass = 7.9577e-3;  //低通滤波系数 20hz

float Filter_Altitude=0.0f;//气压计和超声波和加速度计滤波得到的高度，单位cm
float Filter_Altitude_D = 0;//高度变化率，即垂直速度，单位cm/s
volatile float evaluateAltitude = 0;
float ALT_Update_Interval = 0.0; //两次高度测量，之间的时间间隔
float D2_Alt = 0;//对高度去两次微分
float FiltAlt_Buff[MOVAVG_SIZE] = {0};
uint8_t FiltAlt_index = 0;

void Altitude_Get_D(void);
void FiltAlt_NewAlt(float val);
float FiltAlt_getAvg(float *buff, int size);

void Get_Filter_Altitude(void)
{
    float altitude;
    if(Ultra_Healthy>15)//使用超声波为主
    {
        if (Ultra_ALT_Updated == 1)
        {
            evaluateAltitude = (Filter_Altitude_D + D2_Alt) * ALT_Update_Interval + Filter_Altitude;
            if ((evaluateAltitude + 5.0f) >= altitude && (evaluateAltitude - 5.0f) <= altitude)
            {
                altitude = 90.0f * Ultra_Distance + 10.0f * Position_Z;
            }
            else
            {
                if (MS5611_ALT_Updated == 1)
                {
                    altitude = 1.0f * Ultra_Distance + 98.0f * Position_Z + 0.01f * MS5611_Altitude;
                    MS5611_ALT_Updated = 0;
                }
                else
                {
                    altitude = 1.0f * Ultra_Distance + 99.0f * Position_Z;
                }
            }
            Ultra_ALT_Updated = 0;
        }
        else
            return;
    }
    else//使用气压计
    {
        if(MS5611_ALT_Updated == 1)
        {
            altitude = 0.15f * MS5611_Altitude + 85.0f * Position_Z;
            MS5611_ALT_Updated = 0;
        }
        else
            return;
    }
    Filter_Altitude = Filter_Altitude + (ALT_Update_Interval / (ALT_Update_Interval + FiltAlt_Lowpass)) * (altitude - Filter_Altitude);
    FiltAlt_NewAlt(altitude);
    Altitude_Get_D();
}

void Altitude_Get_D(void)
{
	float new=0,old=0;
	float temp;
	static float D = 0;
	int16_t i;
	for(i=0;i<MOVAVG_SIZE/2;i++)
		old += FiltAlt_Buff[i];
	old /= (MOVAVG_SIZE/2);
	for(i=MOVAVG_SIZE/2;i<MOVAVG_SIZE;i++)
	    new += FiltAlt_Buff[i];
	new /= (MOVAVG_SIZE/2);
	temp = new - old;
	temp = temp * 2.0f / MOVAVG_SIZE / ALT_Update_Interval;
	//对是否会出现0/0做判断，因为在起始的时候，new和old和ALT_Update_Interval都是0，要等好长一段时间后才有值
	if (!isnan(temp))//判断temp是不是一个数字
	{
	    D2_Alt = D;
	    D = D +	(ALT_Update_Interval / (ALT_Update_Interval + FiltAlt_Lowpass)) * (temp - D);
	    D2_Alt = (D - D2_Alt);
	}
	Filter_Altitude_D = 0.75f * D + 25.0f * Motion_Velocity_Z;
	Motion_Velocity_Z = Filter_Altitude_D / 100.0f;
}

void FiltAlt_NewAlt(float val)
{
    static uint32_t last_time = 0;
    uint32_t temp;
    temp = micros();
    if (temp <= last_time)
		ALT_Update_Interval = ((float)(temp + (0xffffffff - last_time))) / 1000000.0f;
    else
		ALT_Update_Interval = ((float)(temp - last_time)) / 1000000.0f;
    last_time = temp;

    FiltAlt_Buff[FiltAlt_index] = val;
    FiltAlt_index = ((FiltAlt_index + 1) % MOVAVG_SIZE);
}

float FiltAlt_getAvg(float * buff, int size) {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return (sum / (float)size);
}


