/**
  ******************************************************************************
  * @file    usart.c
  * @author  佟源洋
  * @version V1.0
  * @date    1-April-2014
  * @brief   This file includes the driver for STM32F407VGT6's usart
	* @attention:          
						 1、使用stm32f4discovery时，不能使用USART1，USART1的PA9脚已经作为VBUS_FS使用，强行使用串口会出现乱码。
							  如若使用串口，请使用其他串口。参见stm32f4 dicovery原理图。
						 2、stm32f4固件库默认HSE_VALUE为25MHZ，而板载晶振为8MHZ,需修改stm32f4xx_conf.h，重定义HSE_VALUE为8MHZ。否则串口也会出现乱码。
								#if defined  (HSE_VALUE) 
								#undef HSE_VALUE 
								#define HSE_VALUE    ((uint32_t)8000000) 
								#endif
xaing：佟源祥代码中usart2用于和PC通信，usart3用于和摄像头连接
  ******************************************************************s************
*/
#include "UART4.h"
#include "delay.h"
#include "camera.h"
#include <stdio.h>
#include "UART1.h"
unsigned char Report_Buf[UART4_RECEIVE_BUFSIZE] = "";

void Initial_UART4(uint32_t bound)
{   
	//uart4在captain飞控板上挂载在PC10 TX和PC11 RX上
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	/* Connect USART pins to AF8 */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);//UART4_TX
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);//UART4_RX
	/*UART4_TX Configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	/*UART4_RX Configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	  
	
    /*配置UART*/
    USART_InitStructure.USART_BaudRate=bound;
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;
    USART_InitStructure.USART_StopBits=USART_StopBits_1;
    USART_InitStructure.USART_Parity=USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_Init(UART4, &USART_InitStructure);
    USART_Cmd(UART4, ENABLE);
    USART_ITConfig(UART4, USART_IT_RXNE,DISABLE); //用查询方式接受所有失能USAT1中断
    USART_ClearFlag(UART4,USART_FLAG_TC);

}

void uart4_putchar(unsigned char ch) //发送一个字符
{  
    USART_ClearFlag(UART4,USART_FLAG_TC);
    USART_SendData(UART4,ch);
    while( USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET );

}

void UART4Write(unsigned char *data,uint32_t len)	 //发送n个字符
{
    uint16_t i;
    if( len != 0)
    { 
        for( i = 0; i < len; i++ )
        {
            uart4_putchar(data[i]);
        }
    }
}

//等待接收数据 接收到的数据，接收到的个数等待接收的时间
uint16_t UART4_Receiver_buf(unsigned char *data,uint32_t recvLen,uint32_t time)
{
    char string_to_send[100] = {0};
    uint16_t count = 0;
    // double waitTime = 0;
    // uint32_t waitTimeLast = micros();
    // uint32_t waitTimeNow;
    uint32_t startTime = micros();
    while (1)
    {
        // waitTimeNow = micros();
        // if (waitTimeNow <= waitTimeLast)
        //     waitTime += ((double)(waitTimeNow + (0xffffffff - waitTimeLast))) / 1000000.0;
        // else
            // waitTime += ((double)(waitTimeNow - waitTimeLast)) / 1000000.0;

        // sprintf(string_to_send, "now %d last %d\r\n", waitTimeNow,waitTimeLast);
        // UART1_Put_String((unsigned char *)string_to_send);
        // waitTimeLast = waitTimeNow;

        if (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) != RESET)
        {
            data[count] = USART_ReceiveData(UART4);
            count = count + 1;
            // sprintf(string_to_send, "0recv %d count %d\r\n", recvLen,count);
            UART1_Put_String((unsigned char *)string_to_send);
            if (count >= recvLen )
            {
                break;
            }
        }
        // else if (waitTime > (double)((time) / 1000000.0))
        else if (micros()-startTime>=time)
        {

            // sprintf(string_to_send, "1time %d count %d len %d\r\n", micros()-startTime, count,recvLen);
            UART1_Put_String((unsigned char *)string_to_send);
            return count;
        }
    }
            // sprintf(string_to_send, "2time %d count %d len %d\r\n", micros()-startTime, count,recvLen);
        UART1_Put_String((unsigned char *)string_to_send);
    return count;
}



