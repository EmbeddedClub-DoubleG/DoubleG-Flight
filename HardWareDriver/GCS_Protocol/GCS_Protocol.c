/* 
占用STM32 资源：
1. 使用UART1发送数据

------------------------------------
 */

#include "GCS_Protocol.h"
#include "USART1DMATX.h"

//xiang:如果要改成自己的上位机和数据协议，注意把每个函数都在项目中查找一下看哪里调用了，最好是直接把这个函数注释掉看哪里报错

#if Yingzhang_GCS
/**************************实现函数********************************************
*函数原型:		void UART1_ReportHeight(float height)
*功　　能:		uart1发送高度数据
输入参数：		高度
xiang:这个函数是我自己写的，针对自制上位机
*******************************************************************************/
void UART1_ReportHeight(float height)//单位cm
{
	char data_to_send[20]={0};
	uint8_t index=0;
	// sprintf(data_to_send,"%d",(int32_t)(height*1000.0f));//乘1000的原因是如果要发送的数据太少（好像是少于4byte），dma就不会把数据发送出去
	sprintf(data_to_send,"%f",height);
	while(data_to_send[index]!='\0')index++;	
	USART1WriteDataToBuffer((unsigned char*)data_to_send,index);

//	i_bytes data;
//	char ctemp;
//	int index = 0;
//	unsigned char data_to_send[5];
//	data.value = (int16_t)(height*1000);
//	ctemp=data.byte[0];
//	data_to_send[index++] = (ctemp);
//	ctemp=data.byte[1];
//	data_to_send[index++] = (ctemp);
//	USART1WriteDataToBuffer(data_to_send,index);
}

void UART1_ReportTHR(void)
{
	char data_to_send[5]={0};
	sprintf(data_to_send,"%04d",THROTTLE);
	USART1WriteDataToBuffer((unsigned char*)data_to_send,4);
}

/**************************实现函数********************************************
*函数原型:		
*功　　能:	发送高度相关PID状态数据到营长GCS	
xiang:这个函数是我自己写的，针对自制上位机
*******************************************************************************/
void UART1_Monitor_AutoHigh(void)//单位cm
{
	char data_to_send[43]={0};
	int32_t data[7];
	data[0] = (int32_t)(Math_fConstrain(AutoHigh_THR.current, -999.99f, 9999.99f) * 100);
	data[1] = (int32_t)(Math_fConstrain(AutoHigh_THR.target, -999.99f, 9999.99f) * 100);
	data[2] = (int32_t)(Math_fConstrain(Climb.current, -999.99f, 9999.99f) * 100);
	data[3] = (int32_t)(Math_fConstrain(Climb.target, -999.99f, 9999.99f) * 100);
	data[4] = (int32_t)(Math_fConstrain(Climb.PID_out, -99.999f, 999.999f) * 1000);
	data[5] = (int32_t)(Math_fConstrain(Increas_Output_Accumulat, -999.99f, 9999.99f) * 100);
	data[6] = (int32_t)(Math_fConstrain(THROTTLE, -999.99f, 9999.99f) * 100);
	sprintf(data_to_send, "%06d%06d%06d%06d%06d%06d%06d", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
	USART1WriteDataToBuffer((unsigned char*)data_to_send,42);
}
#endif

#if Captain_GCS
/**************************实现函数********************************************
*函数原型:	void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
*功　　能:  将姿态数据发送到PC
数据帧格式：
起始		2字节	0xA5 0x5A	：帧起始字节，共两个字节做为起始标志，一个帧发送的第一字节为0xA5 紧接着是0x5A
本帧字节数	1字节				：帧包含的字节个数  计算方法为，除了起始字节(A5 5A)外所有的字节（包含它本身和结束字节）的总数
帧功能字节	1字节				：帧功能字节， 用于标识该帧所指定的帧类型。
数据区							：数据区。每个数据有高位和低位组成，高位在前，低位在后
帧校验字节	1字节				：协议采用累加合校验，所有的数据（除了两个起始字节外），进行累加，最后取低8位做为校验结果。
帧结束字节	1字节	0xAA
*******************************************************************************/
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;	//累加和校验，预先算好帧功能字节和 帧总字节数的校验
	char ctemp;
	int index = 0;
	unsigned char data_to_send[20];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (14+4);
	data_to_send[index++] = (0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=yaw;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=pitch;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=roll;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=alt;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=tempr;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=press;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index);
}
/**************************实现函数********************************************
*函数原型:	void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,int16_t hx,int16_t hy,int16_t hz)
*功　　能:  将传感器数据发送到PC
数据帧格式：
起始		2字节	0xA5 0x5A	：帧起始字节，共两个字节做为起始标志，一个帧发送的第一字节为0xA5 紧接着是0x5A
本帧字节数	1字节				：帧包含的字节个数  计算方法为，除了起始字节(A5 5A)外所有的字节（包含它本身和结束字节）的总数
帧功能字节	1字节				：帧功能字节， 用于标识该帧所指定的帧类型。
数据区							：数据区。每个数据有高位和低位组成，高位在前，低位在后
帧校验字节	1字节				：协议采用累加合校验，所有的数据（除了两个起始字节外），进行累加，最后取低8位做为校验结果。
帧结束字节	1字节	0xAA
*******************************************************************************/
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[30];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (14+8);
	data_to_send[index++] = (0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=ax;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=ay;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=az;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=gx;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=gy;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=gz;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=hx;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=hy;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=hz;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index); 
}

#elif Yingzhang_GCS
/**************************实现函数********************************************
*函数原型:		
*功　　能:	发送当前姿态角以及9轴传感器数据到营长GCS	
xiang:这个函数是我自己写的，针对自制上位机
*******************************************************************************/
void UART1_ReportIMUMotion(float yaw,float pitch,float roll,
					int16_t ax,int16_t ay,int16_t az,
					int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz,
					float alt,float tempr,float press ){
	char data_to_send[91]={0};
	int32_t data[15];
	data[0] = (int32_t)(Math_fConstrain(yaw, -180.00f, 180.00f) * 100);
	data[1] = (int32_t)(Math_fConstrain(roll, -180.00f, 180.00f) * 100);
	data[2] = (int32_t)(Math_fConstrain(pitch, -180.0f, 180.0f) * 100);
	data[3] = (int32_t)(Math_fConstrain(ax, -99999.0f, 999999.0f) * 1);
	data[4] = (int32_t)(Math_fConstrain(ay, -99999.0f, 999999.0f) * 1);
	data[5] = (int32_t)(Math_fConstrain(az, -99999.0f, 999999.0f) * 1);
	data[6] = (int32_t)(Math_fConstrain(gx, -99999.0f, 999999.0f) * 1);
	data[7] = (int32_t)(Math_fConstrain(gy, -99999.0f, 999999.0f) * 1);
	data[8] = (int32_t)(Math_fConstrain(gz, -99999.0f, 999999.0f) * 1);
	data[9] = (int32_t)(Math_fConstrain(hx, -99999.0f, 999999.0f) * 1);
	data[10] = (int32_t)(Math_fConstrain(hy, -99999.0f, 999999.0f) * 1);
	data[11] = (int32_t)(Math_fConstrain(hz, -99999.0f, 999999.0f) * 1);
	data[12] = (int32_t)(Math_fConstrain(alt, -999.99f, 9999.99f) * 100);
	data[13] = (int32_t)(Math_fConstrain(tempr, -99.999f, 999.999f) * 1000);
	data[14] = (int32_t)(Math_fConstrain(press, 0.0f, 999999.0f) * 1);
	sprintf(data_to_send, "%06d%06d%06d%06d%06d%06d%06d%06d%06d%06d%06d%06d%06d%06d%06d", data[0], data[1], data[2], data[3], data[4], data[5],
																						  data[6], data[7], data[8], data[9], data[10], data[11],
																			 				data[12], data[13], data[14]);
	USART1WriteDataToBuffer((unsigned char*)data_to_send,90);
}
#endif

#if Captain_GCS
//将磁力计标定的值发送到PC
void UART1_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz
,int16_t minx,int16_t miny,int16_t minz,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2+3;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[22];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (14+4);
	data_to_send[index++] = (0xA4);

	if(maxx<0)maxx=32768-maxx;
	ctemp=maxx>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=maxx;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(maxy<0)maxy=32768-maxy;
	ctemp=maxy>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=maxy;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(maxz<0)maxz=32768-maxz;
	ctemp=maxz>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=maxz;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(minx<0)minx=32768-minx;
	ctemp=minx>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=minx;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(miny<0)miny=32768-miny;
	ctemp=miny>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=miny;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	if(minz<0)minz=32768-minz;
	ctemp=minz>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=minz;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256);
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index); 
}
#elif Yingzhang_GCS
void UART1_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz,
					int16_t minx,int16_t miny,int16_t minz){
	char data_to_send[37]={0};
	int32_t data[6];
	data[0] = (int32_t)(Math_fConstrain(maxx, -99999.0f, 999999.0f) * 1);
	data[1] = (int32_t)(Math_fConstrain(maxy, -99999.0f, 999999.0f) * 1);
	data[2] = (int32_t)(Math_fConstrain(maxz, -99999.0f, 999999.0f) * 1);
	data[3] = (int32_t)(Math_fConstrain(minx, -99999.0f, 999999.0f) * 1);
	data[4] = (int32_t)(Math_fConstrain(miny, -99999.0f, 999999.0f) * 1);
	data[5] = (int32_t)(Math_fConstrain(minz, -99999.0f, 999999.0f) * 1);
	sprintf(data_to_send, "%06d%06d%06d%06d%06d%06d", data[0], data[1], data[2], data[3], data[4], data[5]);
	USART1WriteDataToBuffer((unsigned char*)data_to_send,36);
}
#endif

#if Captain_GCS
//将PWM输入的脉宽值发送到PC
void UART1_Report_PWMin(void)
{
 	unsigned int temp=0x12+16;
	uint16_t  tempCH;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[20];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (16);
	data_to_send[index++] = (0x12);

	tempCH = PWM_Input_CH1;
	ctemp=tempCH>>8;	//通道1 高8位
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp = tempCH;  //通道1 低8位
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	tempCH = PWM_Input_CH2;
	ctemp=tempCH>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=tempCH;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	tempCH = PWM_Input_CH3;
	ctemp=tempCH>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=tempCH;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	tempCH = PWM_Input_CH4;
	ctemp=tempCH>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=tempCH;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	tempCH = PWM_Input_CH5;
	ctemp=tempCH>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=tempCH;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	//-------------------------
	tempCH = PWM_Input_CH6;
	ctemp=tempCH>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=tempCH;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index); 
}

//将PWM输出 发送到PC
void UART1_Report_PWMout(void)
{
 	unsigned int temp=0x13+20;
	uint16_t  data;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[28];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (20);
	data_to_send[index++] = (0x13);

	data = PWM_Output_CH1;//输出通道1 
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH2;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH3;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH4;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH5;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH6;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH7;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data = PWM_Output_CH8;
	ctemp=data>>8;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data;
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index); 
}
#elif Yingzhang_GCS
//将PWM输入和输出的脉宽值发送到PC
void UART1_Report_PWMInOut(void)
{
	char data_to_send[41]={0};
	int32_t data[10];
	data[0] = (int32_t)(Math_fConstrain(PWM_Input_CH1, 800.0f, 2200.0f) * 1);
	data[1] = (int32_t)(Math_fConstrain(PWM_Input_CH2, 800.0f, 2200.0f) * 1);
	data[2] = (int32_t)(Math_fConstrain(PWM_Input_CH3, 800.0f, 2200.0f) * 1);
	data[3] = (int32_t)(Math_fConstrain(PWM_Input_CH4, 800.0f, 2200.0f) * 1);
	data[4] = (int32_t)(Math_fConstrain(PWM_Input_CH5, 800.0f, 2200.0f) * 1);
	data[5] = (int32_t)(Math_fConstrain(PWM_Input_CH6, 800.0f, 2200.0f) * 1);
	data[6] = (int32_t)(Math_fConstrain(PWM_Output_CH1, 800.0f, 2200.0f) * 1);
	data[7] = (int32_t)(Math_fConstrain(PWM_Output_CH2, 800.0f, 2200.0f) * 1);
	data[8] = (int32_t)(Math_fConstrain(PWM_Output_CH3, 800.0f, 2200.0f) * 1);
	data[9] = (int32_t)(Math_fConstrain(PWM_Output_CH4, 800.0f, 2200.0f) * 1);
	sprintf(data_to_send, "%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
	USART1WriteDataToBuffer((unsigned char*)data_to_send,40);
}
#endif


#if Captain_GCS
//响应上位机读取设置项
void UART1_Return_Setting(void){
	unsigned int temp = 0xB1+11;
	f_bytes data;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[15];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (11);
	data_to_send[index++] = (0xB1);
	data_to_send[index++] = (Config.fly_mode);
	data_to_send[index++] = (Config.GPS_baud);
	data_to_send[index++] = (Config.yaw_dir);
	temp += Config.fly_mode;
	temp += Config.GPS_baud;
	temp += Config.yaw_dir;

	data.value = Config.target_hight;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index); 
}

/**************************实现函数********************************************
*函数原型:		UART1_ReportSysteminfo
*功　　能:		向上位机发送控制器当前的系统信息
输入参数：
int8_t mode,  工作模式。
			  0x01 手动模式	
			  0x02  平衡模式
			  0x03  定点飞行
int16_t bat_vol, 动力电池电压值，单位0.1V  
int16_t sevor_vol, 舵机电源电压值，单位 0.1V
int8_t error_code, 错误代码   自己定义。0-255
int16_t I2C_error_count, I2C总线错误的次数，从上电时开始计
int16_t	system_time  系统时间，单位S 秒。从上电时开始计时 
输出参数：没有	
*******************************************************************************/
void UART1_ReportSysteminfo(
						int8_t mode,
						int16_t bat_vol,
						int16_t sevor_vol,
						int8_t error_code,
						int16_t I2C_error_count,
						int16_t	system_time   //S
						){
	char ctemp = 0,i;
	int index = 0;
	unsigned char data_to_send[25];
	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (14);  // 帧字节数，从这个字节开始到帧结束的总字节，不包含 A5 5A
	data_to_send[index++] = (0xA6);  // 帧标识  帧关键字

	data_to_send[index++] = mode;
	data_to_send[index++] = bat_vol >> 8;
	data_to_send[index++] = bat_vol;
	data_to_send[index++] = sevor_vol >> 8;
	data_to_send[index++] = sevor_vol;
	data_to_send[index++] = error_code;
	data_to_send[index++] = I2C_error_count >> 8;
	data_to_send[index++] = I2C_error_count;

	data_to_send[index++] = system_time >> 8;
	data_to_send[index++] = system_time;
	ctemp = 0;
	for( i = 2;i<index;i++){
			ctemp += data_to_send[i];
			}
	data_to_send[index++] = ctemp;
	data_to_send[index++] = 0xaa;
	USART1WriteDataToBuffer(data_to_send,index);	 
}

/**************************实现函数********************************************
*函数原型:		UART1_ReportPosition
*功　　能:		向上位机发送控制器当前的位置信息  也就是GPS数据
输入参数：
int32_t lon,    经度值，单位0.000001度。当传送的值为 123456789  表示 123.456789度
int32_t lat,    纬度值，单位0.000001度。当传送的值为 12345678  表示  12.345678度
int16_t hight,  GPS海拔高度值，单位0.1米。当传送值为 1623  表示当前海拔为 162.3米
int8_t  STnum,  锁定的卫星数量， 0 表示没有定位、
int16_t heading, GPS航向值，单位0.1度。当传送值为 125时，表示12.5度。
int16_t	speed    GPS速度，单位0.1米/S  当传送的值为 255时，表示 25.5M/S
输出参数：没有	
*******************************************************************************/
void UART1_ReportPosition(
						int32_t lon,
						int32_t lat,
						int16_t hight,
						int8_t  STnum,
						int16_t heading,
						int16_t	speed
						){
	char ctemp = 0,i;
	int index = 0;
	unsigned char data_to_send[21];
	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (19); // 帧字节数，从这个字节开始到帧结束的总字节，不包含 A5 5A
	data_to_send[index++] = (0xA3); // 帧标识  帧关键字

	data_to_send[index++] = lon >> 24;
	data_to_send[index++] = lon >> 16;
	data_to_send[index++] = lon >> 8;
	data_to_send[index++] = lon >> 0;

	data_to_send[index++] = lat >> 24;
	data_to_send[index++] = lat >> 16;
	data_to_send[index++] = lat >> 8;
	data_to_send[index++] = lat >> 0;

	data_to_send[index++] = hight >> 8;
	data_to_send[index++] = hight;

	data_to_send[index++] = STnum;

	data_to_send[index++] = heading >> 8;
	data_to_send[index++] = heading;

	data_to_send[index++] = speed >> 8;
	data_to_send[index++] = speed;
	ctemp = 0;
	for( i = 2;i<index;i++){
			ctemp += data_to_send[i];
			}
	data_to_send[index++] = ctemp;
	data_to_send[index++] = 0xaa;   //帧结束字节
	USART1WriteDataToBuffer(data_to_send,index); 
}

/**************************实现函数********************************************
*函数原型:		UART1_ReportTarget
*功　　能:		向上位机发送控制器当前的目标航点信息  
输入参数：
int32_t lon,    经度值，单位0.000001度。当传送的值为 123456789  表示 123.456789度
int32_t lat,    纬度值，单位0.000001度。当传送的值为 12345678  表示  12.345678度
int16_t hight,  GPS海拔高度值，单位0.1米。当传送值为 1623  表示当前海拔为 162.3米
int16_t Distance, 与目标点的距离 单位 0.1米 
int16_t Xspeed,   通过运算后控制器输出，在X轴上的控制量，单位0.1度 
int16_t	Yspeed    通过运算后控制器输出，四轴在Y轴上的控制量，单位0.1度 
输出参数：没有	
*******************************************************************************/
void UART1_ReportTarget(
						int32_t lon,
						int32_t lat,
						int16_t hight,
						int16_t Distance,
						int16_t Xspeed,
						int16_t	Yspeed
						){
	char ctemp = 0,i;
	int index = 0;
	unsigned char data_to_send[22];
	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (20);	  // 帧字节数，从这个字节开始到帧结束的总字节，不包含 A5 5A
	data_to_send[index++] = (0xA5); // 帧标识  帧关键字

	data_to_send[index++] = lon >> 24;
	data_to_send[index++] = lon >> 16;
	data_to_send[index++] = lon >> 8;
	data_to_send[index++] = lon >> 0;

	data_to_send[index++] = lat >> 24;
	data_to_send[index++] = lat >> 16;
	data_to_send[index++] = lat >> 8;
	data_to_send[index++] = lat >> 0;

	data_to_send[index++] = hight >> 8;
	data_to_send[index++] = hight;

	data_to_send[index++] = Distance >> 8;
	data_to_send[index++] = Distance;

	data_to_send[index++] = Xspeed >> 8;
	data_to_send[index++] = Xspeed;

	data_to_send[index++] = Yspeed >> 8;
	data_to_send[index++] = Yspeed;

	ctemp = 0;
	for( i = 2;i<index;i++){
			ctemp += data_to_send[i];  //累加和校验
			}
	data_to_send[index++] = ctemp;
	data_to_send[index++] = 0xaa; //帧结束字节 	
	USART1WriteDataToBuffer(data_to_send,index); 
}
#endif

#if Captain_GCS
//监控PID状态
void UART1_Report_PID(float target,float current,float pidcon)
{
 	unsigned int temp=0x15+16;
	f_bytes data;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[20];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (16);
	data_to_send[index++] = (0x15);

	data.value = target;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = current;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = pidcon;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index); 
}
#endif

#if Captain_GCS
//将PID参数返回给上位机	  将两组关联的PID发送给上位机
void UART1_Return_PID(struct Quad_PID *PID1,struct Quad_PID *PID2,unsigned char pindex){
  	unsigned int temp=0xB3+pindex+28;
	f_bytes data;
	char ctemp;
	int index = 0;
	unsigned char data_to_send[32];

	data_to_send[index++] = (0xa5);
	data_to_send[index++] = (0x5a);
	data_to_send[index++] = (28);
	data_to_send[index++] = (0xB3+pindex);

	data.value = PID1->Kp;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = PID1->Ki;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = PID1->Kd;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = PID2->Kp;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = PID2->Ki;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data.value = PID2->Kd;
	ctemp=data.byte[0];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	data_to_send[index++] = (ctemp);
	temp+=ctemp;

	data_to_send[index++] = (temp%256); //校验
	data_to_send[index++] = (0xaa);
	USART1WriteDataToBuffer(data_to_send,index);	 
}
#elif Yingzhang_GCS
//将PID参数返回给上位机	  将两组关联的PID发送给上位机
void UART1_Return_PID(struct Quad_PID *PID1,struct Quad_PID *PID2)
{
	char data_to_send[37]={0};
	int32_t data[6];
	data[0] = (int32_t)(Math_fConstrain(PID1->Kp, -0.99999f, 9.99999) * 100000);
	data[1] = (int32_t)(Math_fConstrain(PID1->Ki, -0.99999f, 9.99999) * 100000);
	data[2] = (int32_t)(Math_fConstrain(PID1->Kd, -0.99999f, 9.99999) * 100000);
	data[3] = (int32_t)(Math_fConstrain(PID2->Kp, -0.99999f, 9.99999) * 100000);
	data[4] = (int32_t)(Math_fConstrain(PID2->Ki, -0.99999f, 9.99999) * 100000);
	data[5] = (int32_t)(Math_fConstrain(PID2->Kd, -0.99999f, 9.99999) * 100000);
	sprintf(data_to_send, "%06d%06d%06d%06d%06d%06d", data[0], data[1], data[2], data[3], data[4], data[5]);
	USART1WriteDataToBuffer((unsigned char*)data_to_send,36);
}
#endif
//------------------End of File----------------------------
