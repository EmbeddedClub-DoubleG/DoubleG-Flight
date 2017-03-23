/* GPS.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-10-05
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用STM32 资源：
1. 使用UART3接收GPS数据
2. 确认GPS的波特率 在初始化时调用 Initial_UART3

功能：
接收并提取GPS数据。
------------------------------------
 */

#include "GPS.h"
#include "LED.h"
#include "GCS_Protocol.h"
#include "math.h"
#include "delay.h"


//接收缓冲区
volatile unsigned char GPS_buffer[256];
volatile unsigned char GPS_wr_index;
volatile unsigned char Frame_End,GPS_Data_Ready=0;
//----------------------------------
volatile unsigned char GPS_Satel[4];
volatile unsigned char GPS_Status;
volatile unsigned char GPS_Latitude[11],Lat;
volatile unsigned char GPS_Longitude[13],Lon;
volatile unsigned char GPS_Speed[7];
volatile unsigned char GPS_Course[7];
volatile unsigned char GPS_Height[9]; 
volatile unsigned char GPS_Time[8];
volatile unsigned char GPS_Date[8];
//----------------------------------
unsigned char GPS_STA_Num = 0,
			  GPS_Update = 0, 
			  GPS_Locked = 0;
double GPS_Altitude = 0,
	  Latitude_GPS = 0,
	  Longitude_GPS = 0, 
	  Speed_GPS = 0,
	  Course_GPS = 0;
//---------家点信息---------------------
unsigned char Home_Ready = 0;
float Home_Latitude = 0,Home_Longitude = 0,Home_Altitude = 0;
void UBLOX_GPS_initial(void);

/**************************实现函数********************************************
*函数原型:		void UART3NVIC_Configuration(void)
*功　　能:		初始化USART3中断优先级
*******************************************************************************/
void UART3NVIC_Configuration(void){

    NVIC_InitTypeDef NVIC_InitStructure; 
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 13;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void Initial_UART3(u32 baudrate)
*功　　能:		初始化USART3接口
输入参数：
		u32 baudrate   设置波特率
输出参数：没有	
*******************************************************************************/
void Initial_UART3(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 使能 UART3 模块的时钟  使能 UART3对应的引脚端口PB的时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  	 /* 配置UART3 的发送引脚
	 配置PB10 为复用输出  刷新频率50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);    
  	/* 
	  配置UART3 的接收引脚
	  配置PB11为浮地输入 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	/* 
	  UART3的配置:
	  1.波特率为调用程序指定的输入 baudrate;
	  2. 8位数据			  USART_WordLength_8b;
	  3.一个停止位			  USART_StopBits_1;
	  4. 无奇偶效验			  USART_Parity_No ;
	  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
	  6.使能接收功能	  USART_Mode_Rx ;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //开启接收功能	和发送功能
	//应用配置到UART3
	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	//使能接收中断
	
  	USART_Cmd(USART3, ENABLE);//启动UART3
	UART3NVIC_Configuration(); //配置中断优先级
	//UBLOX_GPS_initial();
}

/**************************实现函数********************************************
*函数原型:		void UART3_Put_Char(unsigned char DataToSend)
*功　　能:		通过 UART3 发送一个字节 的数据
输入  
unsigned char DataToSend 要发送的字节
*******************************************************************************/
void UART3_Put_Char(unsigned char DataToSend){
	USART_SendData(USART3, DataToSend);
	//等待发送完成.
  	while (!(USART3->SR & USART_FLAG_TXE));
}

/**************************实现函数********************************************
*函数原型:		void UART3_Send_Buf(unsigned char *buf,unsigned char len)
*功　　能:		通过 UART3 发送几个字节 的数据
unsigned char *buf  存放数据的数组指针
unsigned char len   要发送的字节数
*******************************************************************************/
void UART3_Send_Buf(unsigned char *buf,unsigned char len){
	unsigned char i ;
	for(i=0;i<len;i++,buf++){
			UART3_Put_Char(*buf);
			}
}

/**************************实现函数********************************************
*函数原型:		void UBLOX_GPS_initial(void)
*功　　能:		初始化 UBLOX GPS 模块，以去掉一些没必要的数据，节省CPU的开支 
*******************************************************************************/
void UBLOX_GPS_initial(void){
	u8 index;  //以下指令只对 ublox GPS模块的设置
	u8 ublox_cmd[15]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};

	index=7;
	ublox_cmd[index++]=0x01; //disable GPGLL
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFB;
	ublox_cmd[index++]=0x11;
	UART3_Send_Buf(ublox_cmd,index);

	index=7;
	ublox_cmd[index++]=0x02; //disable GPGSA
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFC;
	ublox_cmd[index++]=0x13;
	UART3_Send_Buf(ublox_cmd,index);
	delay_ms(10);

	index=7;
	ublox_cmd[index++]=0x03; //disable GPGSV
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFD;
	ublox_cmd[index++]=0x15;
	UART3_Send_Buf(ublox_cmd,index);
	delay_ms(10);

	index=7;
	ublox_cmd[index++]=0x05; //disable GPVTG
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFF;
	ublox_cmd[index++]=0x19;
	UART3_Send_Buf(ublox_cmd,index);
	delay_ms(10);

	index=7;
	ublox_cmd[index++]=0x08; //disable GPZDA
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x02;
	ublox_cmd[index++]=0x1F;
	UART3_Send_Buf(ublox_cmd,index);
	delay_ms(10);

	index=3;
	ublox_cmd[index++]=0x08; //enable 4Hz 更新速度 4HZ
	ublox_cmd[index++]=0x06;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFA;
	ublox_cmd[index++]=0x00; 
	ublox_cmd[index++]=0x01;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x01;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x10; 
	ublox_cmd[index++]=0x96;
	UART3_Send_Buf(ublox_cmd,index);
	delay_ms(10);

}

/**************************实现函数********************************************
*函数原型:		void GPS_Decode(unsigned char len)
*功　　能:		将刚刚接收到的帧数据必要的信息提取出来。
输入参数：
		unsigned char len   接收到的字节数
*******************************************************************************/
void GPS_Decode(unsigned char len){
	unsigned char i , data ,j = 0 ,k = 0;
	
	if((GPS_buffer[0]==0x47)&&(GPS_buffer[1]==0x50)&&(GPS_buffer[2]==0x47)&&(GPS_buffer[3]==0x47)&&(GPS_buffer[4]==0x41)){
	// $GPGGA 全球定位数据
	j = 0; 
	for(i = 4; i < len; i++ ){
		data = GPS_buffer[i]; //取数组中的数据
		if(data == ','){
    	j++; //下一个字段
		k = 0;
	}else{ //非逗号
		switch( j ){  
        	case 7:if(k<3){ //使用卫星数量，从00到12（前导位数不足则补0）
			     		GPS_Satel[k++] = data; 
                 		GPS_Satel[k] = 0;
			   			} 
						break;
        	case 9:if(k<8){ //天线离海平面的高度，-9999.9到9999.9米
			     		GPS_Height[k++] = data; 
                 		GPS_Height[k] = 0;
			   			} 
			  			break;
			case 10 :return ; //后面的数据我们不关心，return 退出   
        	default:break;
      		}	//switch 结束
		}	
		}
	}else if((GPS_buffer[0]==0x47)&&(GPS_buffer[1]==0x50)&&(GPS_buffer[2]==0x52)&&(GPS_buffer[3]==0x4D)&&(GPS_buffer[4]==0x43)){
		// $GPRMC 运输定位数据
		j = 0;
		for(i = 4; i < len; i++ ){
			data = GPS_buffer[i]; //取数组中的数据
			if(data == ','){
    			j++; //下一个字段
				k = 0;
			}else{ //非逗号
			switch( j ){ 
					case 1: if(k < 6) //UTC 时间，hhmmss（时分秒）格式
                	 		GPS_Time[k++] = data;
               		 		break;
        			case 2: GPS_Status=data; //定位状态，A=有效定位，V=无效定位
                			break;
        			case 3: if(k < 9) //Latitude，纬度ddmm.mmmm（度分）格式
		         			GPS_Latitude[k++] = data;
                			break;
					case 4: Lat = data; //纬度半球N（北半球）或S（南半球）
                			break;		
        			case 5: if(k < 10) // Longitude，经度dddmm.mmmm（度分）格式
		         			GPS_Longitude[k++] = data; 
               			 	break;
        			case 6: Lon = data;//经度半球E（东经）或W（西经）
                			break;            
       				case 7: if(k < 6){ //地面速率（000.0~999.9节)
				 			GPS_Speed[k++] = data; 
                 			GPS_Speed[k] = 0;
			   				} 
                			break; 
        			case 8: if(k < 6){ //地面航向（000.0~359.9度，以真北为参考基准)
				 			GPS_Course[k++] = data; 
                 			GPS_Course[k] = 0;
			   				} 
                			break;  
        			case 9: if(k<6){ //UTC日期，ddmmyy（日月年）格式
			     			GPS_Date[k++] = data; 
                 			GPS_Date[k] = 0;
			   				} 
		       				break;      
        					default:break;
      					}
    			}
			}
	GPS_Data_Ready = 1;//数据准备好了，提示主程序可以进行提取转换了。
	} //$GPRMC 运输定位数据
}

//------USART3中断子程序-------------------------------------
void USART3_IRQHandler(void){
	unsigned char indata;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
	indata = USART_ReceiveData(USART3);	//取接收到的字节
	if(indata == 0x24){	  //帧命令起始位 NMEA-0183 '$'
		GPS_wr_index = 0;
		Frame_End = 0;
	}else if(indata == 0x0D){ //CR
		Frame_End = 0xff;
	}else if(indata == 0x0A){ //LF
		if(Frame_End != 0x00){ //上一个字节是0x0D
			GPS_Decode(GPS_wr_index);  //解出必要的数据
			Frame_End = 0;
			}
	}else{ //非关键字 
		GPS_buffer[GPS_wr_index++] = indata;  //存入缓冲区		
		Frame_End = 0;
	}
	if(GPS_wr_index == 0xff){
    		GPS_wr_index--;
			}
	/* Clear the USART3 RX interrupt */
  	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}	
}


//
/**************************实现函数********************************************
*函数原型:		float Asc_to_f(volatile unsigned char *str)
*功　　能:		提取字符串中的 有效数字
输入参数：
		unsigned char *str    字符串数组
		返回数组表示的值。  比如字符串 "1230.01"  经过这个程序后，返回浮点的值为1230.01
*******************************************************************************/
float Asc_to_f(volatile unsigned char *str)
{
  signed char temp,flag1,flag2; 
  float value,count;
  flag1 = 1;
  flag2 = 0;
  value = 0;
  count = 1;
  temp = *str;
  while(((*str>='0')&&(*str<='9'))||(*str=='-')||(*str=='.')) //数字或者是符号
  { 
  	temp=*str++;
    if(temp=='-'){ 
	if(flag1)
	   	flag1=-1;
      else
	   return(0x00); //出现两次'-' 结果无效
	}
	else if(temp=='.'){ 
		 flag2=1;	  
	     }
		 else{ 
		   value=value*10+(temp&0x0f);
           if(flag2)
		    	count*=0.1f;
		 }
  }
  value*=count*flag1; //符号位
  return(value);
}

//----------------------------------
unsigned char init = 0;
float last_LON,last_LAT,GPS_Period;
float now_speed[2],	speed_old[2];
uint32_t  GPS_Updata_time = 0 ;
//-----------GPS模块 的线程，需要定时调用-----------
void GPS_Routing(void){
	float temp,tmp;
	uint32_t templ;
	if(GPS_Data_Ready == 0)//有数据需要处理吗？ 该标志在GPS数据接收中断程序中置位
		return; //没有就退出吧。

	GPS_Data_Ready = 0;	// 清标志 
	//LED_Set_Blink(Red,80,80,1); //LED 闪烁表示正在处理GPS数据
	if(GPS_Status == 'A'){ //定位状态，A=有效定位，V=无效定位
		GPS_Locked = 1;
		}else {
		GPS_Locked = 0;
		Home_Ready = 0;
#if Captain_GCS
		UART1_ReportPosition(Longitude_GPS * 10000,
						 Latitude_GPS * 10000,
						 GPS_Altitude * 10,
						 0 ,
						 Course_GPS * 10,
						 Speed_GPS * 10);
#endif
		return; //都没有定位，下面的数据就不要转换了，浪费时间
		}
	//开始提取有效的位置信息
	GPS_STA_Num = Asc_to_f(GPS_Satel); //使用卫星数量，从00到12

	GPS_Altitude = Asc_to_f(GPS_Height); //天线离海平面的高度，-9999.9到9999.9米
	//纬度 [2446.5241]
	temp = (float)(GPS_Latitude[2]&0x0F)*10.0f+(float)(GPS_Latitude[3]&0x0F)+
		   (float)(GPS_Latitude[5]&0x0F)*0.1f+(float)(GPS_Latitude[6]&0x0F)*0.01f+	
		   (float)(GPS_Latitude[7]&0x0F)*0.001f+(float)(GPS_Latitude[8]&0x0F)*0.0001f;
	temp /= 60.0f;  //转成度
	temp += (float)(GPS_Latitude[0]&0x0F)*10.0f+(float)(GPS_Latitude[1]&0x0F);
	Latitude_GPS = temp;
	if(Lat == 'S'){	//S（南半球）
		Latitude_GPS = -Latitude_GPS;
		}
	//经度 [12100.1536]
	temp = (float)(GPS_Longitude[3]&0x0F)*10.0f+(float)(GPS_Longitude[4]&0x0F)+
		   (float)(GPS_Longitude[6]&0x0F)*0.1f+(float)(GPS_Longitude[7]&0x0F)*0.01f+	
		   (float)(GPS_Longitude[8]&0x0F)*0.001f+(float)(GPS_Longitude[9]&0x0F)*0.0001f;
	temp /= 60.0f;  //转成度
	temp += (float)(GPS_Longitude[0]&0x0F)*100.0f+(float)(GPS_Longitude[1]&0x0F)*10.0f+(float)(GPS_Longitude[2]&0x0F);
	Longitude_GPS = temp;
	if(Lon == 'W'){	 //W（西经）
		Longitude_GPS = -Longitude_GPS;
		}
	//速度
	temp = Asc_to_f(GPS_Speed); //地面速率（000.0~999.9节
	Speed_GPS = temp * 0.51444f; //1节＝1海里/小时＝1.852公里/小时 = 0.514米每秒
	//航向
	Course_GPS = Asc_to_f(GPS_Course);
	
	templ = micros();

	if (init) {	 //计算当前 X 方向和 Y方向上的速度 
	GPS_Period = templ - GPS_Updata_time;
	GPS_Period /= 1000000.0f; //转成 秒
    tmp = 1.0f / GPS_Period;	 //速度。
    now_speed[_X] = (Longitude_GPS - last_LON) * cos(((Latitude_GPS  + last_LAT)/2)* 0.0174532925f) * tmp;
    now_speed[_Y] = (Latitude_GPS  - last_LAT)  * tmp;
  
    now_speed[_X] = (now_speed[_X] + speed_old[_X]) / 2;
    now_speed[_Y] = (now_speed[_Y] + speed_old[_Y]) / 2;
  
    speed_old[_X] = now_speed[_X];
    speed_old[_Y] = now_speed[_Y];
    }
    init=1;

	last_LON = Longitude_GPS;
    last_LAT = Latitude_GPS;
	GPS_Updata_time = templ;

	if(GPS_STA_Num > 5){ //锁定超过5个卫星。
			//LED_Set_Blink(Red,1000,100,1); //LED 长亮，表示位置锁定精度较高
			
			if(Home_Ready != 0);
			else{  //取家点。
			Home_Latitude = Latitude_GPS;
			Home_Longitude = Longitude_GPS;
			Home_Altitude = GPS_Altitude;
			Home_Ready = 1;
			}
			}
	//上报当前的位置信息
#if Captain_GCS
	UART1_ReportPosition(Longitude_GPS * 1000000,
						 Latitude_GPS * 1000000,
						 GPS_Altitude * 10,
						 GPS_STA_Num ,
						 Course_GPS * 10,
						 Speed_GPS * 10);
#endif

	GPS_Update = 1; //GPS位置已更新，提示导航数据可用
}

// 清家点标志 ，以便重新定义家点。
void GPS_Reset_Home(void){
	Home_Ready = 0;
}

/*
计算两个点的距离。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回计算出来的距离   单位 米
*/
float GPS_Distance(float lat1,float lon1,float lat2,float lon2){
   	float temp;
	float mLat = (lat2 - lat1)*110946.0f;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f)*111318.0f ;
	temp = 	sqrt(mLat*mLat + mLon*mLon); 	//纬度1度 = 大约111km = 111319.5米
	return temp;
}

/*
计算两个点的连线的 航向角， 以正北为0 。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回 的航向角，单位度。
*/
float GPS_Heading(float lat1,float lon1,float lat2,float lon2){
	float temp;
	float mLat = lat2 - lat1;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
	temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;

	if(temp < 0)temp += 360.0f;
	return temp;
}

//------------------End of File----------------------------
