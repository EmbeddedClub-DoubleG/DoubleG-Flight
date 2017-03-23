/* UART1.C file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-28
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

功能：实现	[Captain 飞控板] 的 UART1 接口操作

---------硬件上的引脚连接:----------
UART1接口：
UART1TXD  -->  PA9  (UART1-TXD)
UART1RXD  -->  PA10 (UART1-RXD)
------------------------------------
 */

#include "common.h"
#include "UART1.h"
#include "USART1DMATX.h"
#include "OSQMem.h"	
#include <stdlib.h>

extern u8 OSUSART1MemQ[OS_MEM_USART1_MAX];  			//空白内存块
OSMEMTcb* OSQUSART1Index;

//接收缓冲区
volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_wr_index;
volatile unsigned char RC_Flag;
unsigned char USART1_DataSize;


void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA2_Stream7 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化STM32-SDK开发板上的RS232接口
输入参数：
		u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void Initial_UART1(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
	u8 err;

	/* 使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  	 /* 配置UART1 的发送引脚
	 配置PA9 为复用输出  刷新频率50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  配置UART1 的接收引脚
	  配置PA10为浮地输入 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	USART_DeInit(USART1);  
	/* 
	  UART1的配置:
	  1.波特率为调用程序指定的输入 baudrate;
	  2. 8位数据			  USART_WordLength_8b;
	  3.一个停止位			  USART_StopBits_1;
	  4. 无奇偶效验			  USART_Parity_No ;
	  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
	  6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//应用配置到UART1
	USART_Init(USART1, &USART_InitStructure); 
	//USART_ITConfig(USART1, USART_IT_TXE, DISABLE);        
    //USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//使能接收中断
	//启动UART1
  	USART_Cmd(USART1, ENABLE);

	//DMA 请求
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream7);
	//USART1 TX DMA Configure
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&OSUSART1MemQ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//这里是byte
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	 // 使能发送DMA
	NVIC_Configuration(); //配置中断优先级
	//申请内存管理管理快
	OSQUSART1Index=(OSMEMTcb *)OSMemCreate(OSUSART1MemQ,OS_MEM_USART1_BLK,OS_MEM_USART1_MAX/OS_MEM_USART1_BLK,&err);
	//DMA_Cmd(DMA2_Stream7, ENABLE);
}

/*******************************************************************************
* 文件名	  	 : DMA2_Stream7_IRQHandler
* 描述	         : DMA2_Stream7_IRQHandler（USART1发送）DMA函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
	{
	    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);	
		USART1DMAUpdate();
	}
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_String(unsigned char *buffer)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *buffer   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART1_Put_String(unsigned char *buffer)
{
	unsigned long count=0;
	while(buffer[count]!='\0')count++;
	(USART1WriteDataToBuffer(buffer,count));	
}

#if Captain_GCS
//提取从上位机发送来的设置项
void UART1_Get_Setting(void){
	f_bytes data;
	Config.fly_mode = rx_buffer[2];
	Config.yaw_dir = rx_buffer[4];
	Config.GPS_baud = rx_buffer[3];
	data.byte[0] = rx_buffer[5];
	data.byte[1] = rx_buffer[6];
	data.byte[2] = rx_buffer[7];
	data.byte[3] = rx_buffer[8];
	Config.target_hight = data.value;
	AT45DB_Write_config();
}
#endif

#if Captain_GCS
//读取PC发送来的PID参数，存到AT45DB
void UART1_Get_PID(struct Quad_PID *PID1,struct Quad_PID *PID2){
	f_bytes data;
	unsigned char i = 2;
	data.byte[0] = rx_buffer[i++];
	data.byte[1] = rx_buffer[i++];
	data.byte[2] = rx_buffer[i++];
	data.byte[3] = rx_buffer[i++];
	PID1->Kp = data.value; 

	data.byte[0] = rx_buffer[i++];
	data.byte[1] = rx_buffer[i++];
	data.byte[2] = rx_buffer[i++];
	data.byte[3] = rx_buffer[i++];
	PID1->Ki = data.value;

	data.byte[0] = rx_buffer[i++];
	data.byte[1] = rx_buffer[i++];
	data.byte[2] = rx_buffer[i++];
	data.byte[3] = rx_buffer[i++];
	PID1->Kd = data.value;

	data.byte[0] = rx_buffer[i++];
	data.byte[1] = rx_buffer[i++];
	data.byte[2] = rx_buffer[i++];
	data.byte[3] = rx_buffer[i++];
	PID2->Kp = data.value;

	data.byte[0] = rx_buffer[i++];
	data.byte[1] = rx_buffer[i++];
	data.byte[2] = rx_buffer[i++];
	data.byte[3] = rx_buffer[i++];
	PID2->Ki = data.value;

	data.byte[0] = rx_buffer[i++];
	data.byte[1] = rx_buffer[i++];
	data.byte[2] = rx_buffer[i++];
	data.byte[3] = rx_buffer[i++];
	PID2->Kd = data.value;

	AT45DB_Write_config();  //把刚收到的PID值存到 AT45DB161 
	//LED_Set_Blink(Blue,50,50,4); //LED 闪烁
}
#elif Yingzhang_GCS
#define charToNum(x) (x-'0')
//读取PC发送来的PID参数，存到AT45DB
void UART1_Get_PID(struct Quad_PID *PID1,struct Quad_PID *PID2)
{//TODO 没写完，字符串转float
//1

	// unsigned char i = 2;
	// int32_t data;

	// data = charToNum(rx_buffer[i++]);
	// i++;
	// data += charToNum(rx_buffer[i++]) / 10;
	// data += charToNum(rx_buffer[i++]) / 100;
	// data += charToNum(rx_buffer[i++]) / 1000;
	// data += charToNum(rx_buffer[i++]) / 10000;
	// data += charToNum(rx_buffer[i++]) / 100000;
	// PID1->Kp = data;

	// data = charToNum(rx_buffer[i++]);
	// i++;
	// data += charToNum(rx_buffer[i++]) / 10;
	// data += charToNum(rx_buffer[i++]) / 100;
	// data += charToNum(rx_buffer[i++]) / 1000;
	// data += charToNum(rx_buffer[i++]) / 10000;
	// data += charToNum(rx_buffer[i++]) / 100000;
	// PID1->Ki = data;

	// data = charToNum(rx_buffer[i++]);
	// i++;
	// data += charToNum(rx_buffer[i++]) / 10;
	// data += charToNum(rx_buffer[i++]) / 100;
	// data += charToNum(rx_buffer[i++]) / 1000;
	// data += charToNum(rx_buffer[i++]) / 10000;
	// data += charToNum(rx_buffer[i++]) / 100000;
	// PID1->Kd = data;

	// data = charToNum(rx_buffer[i++]);
	// i++;
	// data += charToNum(rx_buffer[i++]) / 10;
	// data += charToNum(rx_buffer[i++]) / 100;
	// data += charToNum(rx_buffer[i++]) / 1000;
	// data += charToNum(rx_buffer[i++]) / 10000;
	// data += charToNum(rx_buffer[i++]) / 100000;
	// PID2->Kp = data;

	// data = charToNum(rx_buffer[i++]);
	// i++;
	// data += charToNum(rx_buffer[i++]) / 10;
	// data += charToNum(rx_buffer[i++]) / 100;
	// data += charToNum(rx_buffer[i++]) / 1000;
	// data += charToNum(rx_buffer[i++]) / 10000;
	// data += charToNum(rx_buffer[i++]) / 100000;
	// PID2->Ki = data;

	// data = charToNum(rx_buffer[i++]);
	// i++;
	// data += charToNum(rx_buffer[i++]) / 10;
	// data += charToNum(rx_buffer[i++]) / 100;
	// data += charToNum(rx_buffer[i++]) / 1000;
	// data += charToNum(rx_buffer[i++]) / 10000;
	// data += charToNum(rx_buffer[i++]) / 100000;
	// PID2->Kd = data;

	// AT45DB_Write_config(); //把刚收到的PID值存到 AT45DB161 
//2
	// unsigned char i = 2;
	// unsigned char m;
	// char str[8] = {0};

	// for (m = 0; m < 7;m++)
	// {
	// 	str[m]=rx_buffer[i++];
	// }
	// PID1->Kp = atof(str);

	// for (m = 0; m < 7;m++)
	// {
	// 	str[m]=rx_buffer[i++];
	// }
	// PID1->Ki = atof(str);

	// for (m = 0; m < 7;m++)
	// {
	// 	str[m]=rx_buffer[i++];
	// }
	// PID1->Kd = atof(str);

	// for (m = 0; m < 7;m++)
	// {
	// 	str[m]=rx_buffer[i++];
	// }
	// PID2->Kp = atof(str);

	// for (m = 0; m < 7;m++)
	// {
	// 	str[m]=rx_buffer[i++];
	// }
	// PID2->Ki = atof(str);

	// for (m = 0; m < 7;m++)
	// {
	// 	str[m]=rx_buffer[i++];
	// }
	// PID2->Kd = atof(str);

	// AT45DB_Write_config(); //把刚收到的PID值存到 AT45DB161 
//3
	unsigned char i = 2;
	unsigned char m;
	char str[10] = {0};
	// char strToSend[50] = {0};
	
	for (m = 0; m < 9;m++)
	    str[m] = '\0';
	for (m = 0; m < 9;m++)
	{
		if(rx_buffer[i]==0xbb)
		{
		    i++;
		    break;
		}
		str[m]=rx_buffer[i++];
	}
	PID1->Kp = atof(str)/100000;

	for (m = 0; m < 9;m++)
	    str[m] = '\0';
	for (m = 0; m < 9;m++)
	{
		if(rx_buffer[i]==0xbb)
		{
		    i++;
		    break;
		}
		str[m]=rx_buffer[i++];
	}
	PID1->Ki = atof(str)/100000;

	for (m = 0; m < 9;m++)
	    str[m] = '\0';
	for (m = 0; m < 9;m++)
	{
		if(rx_buffer[i]==0xbb)
		{
		    i++;
		    break;
		}
		str[m]=rx_buffer[i++];
	}
	PID1->Kd = atof(str)/100000;

	for (m = 0; m < 9;m++)
	    str[m] = '\0';
	for (m = 0; m < 9;m++)
	{
		if(rx_buffer[i]==0xbb)
		{
		    i++;
		    break;
		}
		str[m]=rx_buffer[i++];
	}
	PID2->Kp = atof(str)/100000;

	for (m = 0; m < 9;m++)
	    str[m] = '\0';
	for (m = 0; m < 9;m++)
	{
		if(rx_buffer[i]==0xbb)
		{
		    i++;
		    break;
		}
		str[m]=rx_buffer[i++];
	}
	PID2->Ki = atof(str)/100000;

	for (m = 0; m < 9;m++)
	    str[m] = '\0';
	for (m = 0; m < 9;m++)
	{
		if(rx_buffer[i]==0xbb)
		{
		    i++;
		    break;
		}
		str[m]=rx_buffer[i++];
	}
	PID2->Kd = atof(str)/100000;

	//USART1WriteDataToBuffer((unsigned char *)rx_buffer, USART1_DataSize);

	AT45DB_Write_config(); //把刚收到的PID值存到 AT45DB161 
//4
	// unsigned char i = 2;
	// d_bytes data;
	
	// data.byte[0] = rx_buffer[i++];
	// data.byte[1] = rx_buffer[i++];
	// data.byte[2] = rx_buffer[i++];
	// data.byte[3] = rx_buffer[i++];
	// data.byte[4] = rx_buffer[i++];
	// data.byte[5] = rx_buffer[i++];
	// data.byte[6] = rx_buffer[i++];
	// data.byte[7] = rx_buffer[i++];
	// PID1->Kp = data.value/100000;
	// i++;
	
	// data.byte[0] = rx_buffer[i++];
	// data.byte[1] = rx_buffer[i++];
	// data.byte[2] = rx_buffer[i++];
	// data.byte[3] = rx_buffer[i++];
	// data.byte[4] = rx_buffer[i++];
	// data.byte[5] = rx_buffer[i++];
	// data.byte[6] = rx_buffer[i++];
	// data.byte[7] = rx_buffer[i++];
	// PID1->Ki = data.value/100000;
	// i++;
	
	// data.byte[0] = rx_buffer[i++];
	// data.byte[1] = rx_buffer[i++];
	// data.byte[2] = rx_buffer[i++];
	// data.byte[3] = rx_buffer[i++];
	// data.byte[4] = rx_buffer[i++];
	// data.byte[5] = rx_buffer[i++];
	// data.byte[6] = rx_buffer[i++];
	// data.byte[7] = rx_buffer[i++];
	// PID1->Kd = data.value/100000;
	// i++;
	
	// data.byte[0] = rx_buffer[i++];
	// data.byte[1] = rx_buffer[i++];
	// data.byte[2] = rx_buffer[i++];
	// data.byte[3] = rx_buffer[i++];
	// data.byte[4] = rx_buffer[i++];
	// data.byte[5] = rx_buffer[i++];
	// data.byte[6] = rx_buffer[i++];
	// data.byte[7] = rx_buffer[i++];
	// PID2->Kp = data.value/100000;
	// i++;
	
	// data.byte[0] = rx_buffer[i++];
	// data.byte[1] = rx_buffer[i++];
	// data.byte[2] = rx_buffer[i++];
	// data.byte[3] = rx_buffer[i++];
	// data.byte[4] = rx_buffer[i++];
	// data.byte[5] = rx_buffer[i++];
	// data.byte[6] = rx_buffer[i++];
	// data.byte[7] = rx_buffer[i++];
	// PID2->Ki = data.value/100000;
	// i++;
	
	// data.byte[0] = rx_buffer[i++];
	// data.byte[1] = rx_buffer[i++];
	// data.byte[2] = rx_buffer[i++];
	// data.byte[3] = rx_buffer[i++];
	// data.byte[4] = rx_buffer[i++];
	// data.byte[5] = rx_buffer[i++];
	// data.byte[6] = rx_buffer[i++];
	// data.byte[7] = rx_buffer[i++];
	// PID2->Kd = data.value/100000;

	// AT45DB_Write_config(); //把刚收到的PID值存到 AT45DB161 
}
#endif

#if Captain_GCS
/**************************实现函数********************************************
*函数原型:	void USART1_IRQHandler(void)
*功　　能:  uart1数据接收中断
数据帧格式：
起始		2字节	0xA5 0x5A	：帧起始字节，共两个字节做为起始标志，一个帧发送的第一字节为0xA5 紧接着是0x5A
本帧字节数	1字节				：帧包含的字节个数  计算方法为，除了起始字节(A5 5A)外所有的字节（包含它本身和结束字节）的总数
帧功能字节	1字节				：帧功能字节， 用于标识该帧所指定的帧类型。
数据区							：数据区。每个数据有高位和低位组成，高位在前，低位在后
帧校验字节	1字节				：协议采用累加合校验，所有的数据（除了两个起始字节外），进行累加，最后取低8位做为校验结果。
帧结束字节	1字节	0xAA
接收到的数据是从“本帧字节数”这个字节开始存储到rx_buffer里的，起始标志不用存储
*******************************************************************************/
void USART1_IRQHandler(void)
{
	unsigned char data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		data=USART_ReceiveData(USART1);
		if(data==0xa5)//xiang:0xa5是数据开头标识的第一个字节
		{ 
			RC_Flag|=b_uart_head;//xiang：标记已经接收到0xa5
			rx_buffer[rx_wr_index++]=data;//xiang：这个是为了如果下一个字节如果不是0x5a则，这个字节作为数据存储
		}
		else if(data==0x5a)//xiang:0xa5是数据开头标识的第二个字节
		{
			if(RC_Flag&b_uart_head)//xiang：如果已经收到0xa5，则从头开始接收数据
			{
				rx_wr_index=0;
				RC_Flag&=~b_rx_over;//xiang：reset over标志位
			}
			else
				{ rx_buffer[rx_wr_index++]=data; }
			RC_Flag&=~b_uart_head;//xiang：reset head标志位
		}
		else
		{
			rx_buffer[rx_wr_index++]=data;
			RC_Flag&=~b_uart_head;
			if(rx_wr_index==rx_buffer[0])
				RC_Flag|=b_rx_over;
		}
		if(rx_wr_index==RX_BUFFER_SIZE)
			{ rx_wr_index--; }
		/* Clear the USART1 RX interrupt */
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
#elif Yingzhang_GCS
/**************************实现函数********************************************
xiang：这个函数是我自己写的，为了用自己的上位机，用于替代captain原来的函数
*函数原型:	void USART1_IRQHandler(void)
*功　　能:  uart1数据接收中断
数据帧格式：
起始		2字节	0xA5 0x5A	：帧起始字节，共两个字节做为起始标志，一个帧发送的第一字节为0xA5 紧接着是0x5A
本帧字节数	1字节				：帧包含的字节个数  计算方法为，除了起始字节(A5 5A)外所有的字节（包含它本身）的总数
PC_comm		1字节				：PC发送来的命令标识
数据区		若干字节			：如果是一些带有数据的命令，比如写入PID参数，则有数据区
接收到的数据是从“本帧字节数”这个字节开始存储到rx_buffer里的，起始标志不用存储
数据接收结束有个并联的判断条件：接收到0xa50xaa或者接收到的数据长度等于本帧字节数
*******************************************************************************/
void USART1_IRQHandler(void)//xiang：注意：这个函数是针对自己写的上位机的，如果要用captain上位机，就用上面那个函数
{
    unsigned char data;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
		data = USART_ReceiveData(USART1);
		if (data == 0xa5) //xiang:0xa5是数据开头或结尾标识的第一个字节
		{
			RC_Flag |= b_uart_head;	  //xiang：标记已经接收到0xa5
			rx_buffer[rx_wr_index++] = data; //xiang：这个是为了如果下一个字节如果不是0x5a则，这个字节作为数据存储
		}
		else if (data == 0x5a) //xiang:0xa5是数据开头标识的第二个字节
		{
			if (RC_Flag & b_uart_head) //xiang：如果已经收到0xa5，则从头开始接收数据
			{
			rx_wr_index = 0;
			RC_Flag &= ~b_rx_over; //xiang：reset over标志位
			}
			else
			{
			rx_buffer[rx_wr_index++] = data;
			}
			RC_Flag &= ~b_uart_head; //xiang：reset head标志位
		}
		else if(data==0xaa)//xiang：0xaa是数据结尾标识的第二个字节
		{
			if (RC_Flag & b_uart_head) //xiang：如果已经收到0xa5，则从头开始接收数据
			{
				rx_buffer[rx_wr_index++] = data;//把结尾标识也存储起来
				USART1_DataSize = rx_wr_index;
				RC_Flag |= b_rx_over;
			}
			else
			{
			rx_buffer[rx_wr_index++] = data;
			}
			RC_Flag &= ~b_uart_head; //xiang：reset head标志位
		}
		else
		{
			rx_buffer[rx_wr_index++] = data;
			RC_Flag &= ~b_uart_head;
			//TODO 计算数据帧长度
			//计算数据帧长度
			data = rx_buffer[0] / 16;//data在这里只是随便借用的一个临时变量，懒得再声明变量而已
			USART1_DataSize = rx_buffer[0] * 16 + data;
			// sprintf(str, "USART1_DataSize:%d", USART1_DataSize);
			// UART1_Put_String((unsigned char*)str);
			if (rx_wr_index == USART1_DataSize)
			{
				RC_Flag |= b_rx_over;
			}
		}
		if (rx_wr_index == RX_BUFFER_SIZE)
		{
			rx_wr_index--;
		}
		/* Clear the USART1 RX interrupt */
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
#endif

#if Captain_GCS
//校验接收到的数据
unsigned char Sum_check(void)
{ 
  unsigned char i;
  unsigned int checksum=0; 
  for(i=0;i<rx_buffer[0]-2;i++)
   checksum+=rx_buffer[i];  // 累加合校验
  if((checksum%256)==rx_buffer[rx_buffer[0]-2])
   return(0x01); //Checksum successful
  else
   return(0x00); //Checksum error
}
//查询是否接收数据
unsigned char UART1_CommandRoute(void)
{
 if(RC_Flag&b_rx_over){
		RC_Flag&=~b_rx_over;
		if(Sum_check()){
		return rx_buffer[1];
		}
	}
return 0xff; //没有收到上位机的命令，或者是命令效验没有通过
}
#elif Yingzhang_GCS
unsigned char UART1_CommandRoute(void)//xiang：注意：这个函数是针对自己写的上位机的，如果要用captain上位机，就用上面那个函数
{
	if(RC_Flag&b_rx_over)
	{
		RC_Flag&=~b_rx_over;
		return rx_buffer[1];
	}
	return 0xff; //没有收到上位机的命令，或者是命令效验没有通过
}
#endif
//------------------End of File----------------------------
