
#include "common.h"


/**************************实现函数********************************************
*函数原型:		void AT45db161_SPI_Configuration(void)
*功　　能:	    初始化 AT45DB 的SPI 接口
*******************************************************************************/
void SPI3_Configuration(void){

	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	/* SCK, MISO and MOSI  PB3=CLK,PB4=MISO,PB5=MOSI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI3);
	/*  PC8 .9 作片选*/
	GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);//预置为高
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* SPI3 configuration  */
	SPI_Cmd(SPI3, DISABLE);        
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //主
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        //CPOL=0 时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //CPHA=0 数据捕获第1个
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //软件NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //128分频

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
    
	SPI_Init(SPI3, &SPI_InitStructure);	 //应用配置到 SPI3
	SPI_Cmd(SPI3, ENABLE); 
}

/************************************************************************
** 函数名称:uint8_t SPI3_ReadWrite_Byte(uint8_t byte)
** 功能描述:  发送或者接收1个字节
** 输　入:    byte    发送时候,byte传递为发送的数据字节， 接收的时候，则固定为0xff
** 输　出:    SPI3->DR  发送时候，可以忽略, 接收的时候，则为接收数据
***********************************************************************/
uint8_t SPI3_ReadWrite_Byte(uint8_t byte)
{
	/*等待发送寄存器空*/
	while((SPI3->SR & SPI_I2S_FLAG_TXE)==RESET);
	SPI3->DR = byte;  //发送一个字节
	/* 等待接收寄存器有效*/
	while((SPI3->SR & SPI_I2S_FLAG_RXNE)==RESET);
	return(SPI3->DR);
}

//写寄存器
void SPI3_writeReg(u8 reg ,u8 data){
	SPI3_ReadWrite_Byte(reg);
	SPI3_ReadWrite_Byte(data);
}

//读寄存器
u8 SPI3_readReg(u8 reg){
	SPI3_ReadWrite_Byte(reg|0x80);
	return SPI3_ReadWrite_Byte(0xff);
}

//从寄存器读出多个字节[寄存器地址要自动增加]
void SPI3_readRegs(u8 reg, u8 length, u8 *data){
	u8 count = 0;
	SPI3_ReadWrite_Byte(reg|0x80);
	for(count=0;count<length;count++){
		data[count] = SPI3_ReadWrite_Byte(0xff);
	}
}

//------------------End of File----------------------------
