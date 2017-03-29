#include "flow.h"
#define MOVAVG_SIZE  10	   //保存最近的十个数据进行平均滤波
static float Dist_Xspeed_buffer[MOVAVG_SIZE] = {0};
static float Dist_Yspeed_buffer[MOVAVG_SIZE] = {0};
static uint8_t Dis_index = 0;
unsigned char flowdata[47];
short int X_Speed,Y_Speed;
float Xmove=0,Ymove=0;

void Flow_XYspeed_NewDis(float xspeed,float yspeed) 
{
  Dist_Xspeed_buffer[Dis_index] = xspeed;
	Dist_Yspeed_buffer[Dis_index] = yspeed;
  Dis_index = ((Dis_index + 1) % MOVAVG_SIZE);  
}

float Flow_XYspeed_getAvg(float * buff, int size)
{
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return (sum / (float)size);
}


//读指定寄存器指定字节数数据
u8 flow_read_data(u8 addr,u8 reg,u8 len,u8 *buf)
{
    IIC_Start(); 
    IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令 
    if(IIC_Wait_Ack())    //等待应答
    {
        IIC_Stop();         
        return 1;        
    }
    IIC_Send_Byte(reg);    //写寄存器地址
    IIC_Wait_Ack();        //等待应答
    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令    
    IIC_Wait_Ack();        //等待应答 
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据，发送nACK
        else *buf=IIC_Read_Byte(1);        //读数据，发送nACK
        len--;
        buf++; 
    }    
    IIC_Stop();    //产生一个停止条件
    return 0;    
}
//读8位无符号数据
uint8_t     readu8_date(u8 addr,u8 reg)
{
    u8 buff[1];
    uint8_t date;
    flow_read_data(addr,reg,1,buff);
    date = buff[0];
    return date;
}
//读16位无符号数据
uint16_t    readu16_date(u8 addr,u8 reg)
{
    u8 buff[2];
    uint16_t date;
    flow_read_data(addr,reg,2,buff);
    date = buff[1] << 8 | buff[0];
    return date;

}
//读16位有符号数据
int16_t     reads16_date(u8 addr,u8 reg)
{
    u8 buff[2];
    int16_t date;
    flow_read_data(addr,reg,2,buff);
    date = buff[1] << 8 | buff[0];
    return date;
}
//读32位无符号数据
uint32_t    readu32_date(u8 addr,u8 reg)
{
    u8 buff[4];
    int16_t date;
    flow_read_data(addr,reg,2,buff);
    date = buff[3] << 24 | buff[2] << 16 | buff[1] << 8 | buff[0];
    return date;
}
//更新光流数据
void FLOW_getData(void)
{
	short int speed[2];
	uint32_t now_time;
	static uint32_t last_time = 0;
	static short int Last_X_Speed = 0,Last_Y_Speed = 0;
	float dt;

	if(last_time == 0){
		last_time = micros();
		return ;
	}
	now_time = micros(); 
	if(now_time < last_time){ 
		last_time = now_time;
		return ;
	}
	dt = (float)(now_time - last_time);
	dt /= 1000000.0f;  
	last_time=now_time;
	

	flow_read_data(FLOW_ADDR,0,47,flowdata);
	speed[0] = (flowdata[7]<<8)|flowdata[6]; //X轴速度
	speed[1] = (flowdata[9]<<8)|flowdata[8]; //Y轴速度
	Flow_XYspeed_NewDis(speed[0],speed[1]);  //放入滑动队列，取均值
	X_Speed = Flow_XYspeed_getAvg( Dist_Xspeed_buffer, MOVAVG_SIZE);
	Y_Speed = Flow_XYspeed_getAvg( Dist_Yspeed_buffer, MOVAVG_SIZE);
	
	X_Speed = Last_X_Speed + 
	(dt / (15.9155e-3f + dt) * X_Speed - Last_X_Speed);
	Y_Speed = Last_Y_Speed + 
	(dt / (15.9155e-3f + dt) * Y_Speed - Last_Y_Speed);
	Last_X_Speed = X_Speed;
	Last_Y_Speed = Y_Speed;

//	test[0] = (flowdata[23]<<8)|flowdata[22];
//	test[1] = (flowdata[21]<<8)|flowdata[20];
	if((((int16_t)(PWM_Input_CH1 - PWM_Input_Offset) > (int16_t)-35)&&((int16_t)(PWM_Input_CH1 - PWM_Input_Offset) < (int16_t)35))&&(((int16_t)(PWM_Input_CH2 - PWM_Input_Offset) > (int16_t)-35)&&((int16_t)(PWM_Input_CH2 - PWM_Input_Offset) < (int16_t)35)))
	{
		Xmove = Xmove + X_Speed*dt;
		Ymove = Ymove + Y_Speed*dt;
	}
	else
	{
		Xmove = 0;
		Ymove = 0;
	}
}

