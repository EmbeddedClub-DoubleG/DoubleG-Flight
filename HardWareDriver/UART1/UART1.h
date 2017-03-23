#ifndef __UART1_H
#define __UART1_H

#include <stdio.h> 
#include "Quadrotor.h"
#include "stm32f4xx.h"
#include "fly_config.h"

//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40

// USART Receiver buffer
#define RX_BUFFER_SIZE 100

void Initial_UART1(u32 baudrate);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put_String(unsigned char *Str);
//void UART1_SendInt16(int16_t data16);//xiang:这个函数是我自己写的

unsigned char UART1_CommandRoute(void);
#if Captain_GCS
//提取从上位机发送来的设置项
void UART1_Get_Setting(void);
//读取PC发送来的PID参数，存到AT45DB
void UART1_Get_PID(struct Quad_PID *PID1,struct Quad_PID *PID2);
#elif Yingzhang_GCS
void UART1_Get_PID(struct Quad_PID *PID1, struct Quad_PID *PID2);
#endif


// void DMA_Add_buf(void);

#endif

//------------------End of File----------------------------

