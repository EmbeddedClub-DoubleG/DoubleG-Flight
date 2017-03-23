#ifndef _UART4_H
#define _UART4_H
/*******************************INCLUDES******************************/
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>

#define  UART4_RECEIVE_BUFSIZE     2000

#define  second	 1000000

#define  L_TIME  (1*second)
#define  Z_TIME  (10*second)
#define  HH_TIME (90*second)
#define  H_TIME  (30*second) //设置串口接收的等待时间

#define USART_RX_BUF_SIZE	128
#define USART_TX_BUF_SIZE 128

#define USART_STA_RECEIVED  1<<0
#define USART_STA_SENT      1<<1
	
void Initial_UART4(uint32_t bound);
void uart4_putchar(unsigned char ch);
void UART4Write(unsigned char *data,uint32_t len);
uint16_t UART4_Receiver_buf(unsigned char *data,uint32_t recvLen,uint32_t time);


extern unsigned char Report_Buf[UART4_RECEIVE_BUFSIZE];
	
#endif
