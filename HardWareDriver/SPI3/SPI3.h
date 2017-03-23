#ifndef __SPI3_H
#define __SPI3_H

#include "stm32f4xx.h"

void SPI3_Configuration(void);
uint8_t SPI3_ReadWrite_Byte(uint8_t byte);
u8 SPI3_readReg(u8 reg);
void SPI3_writeReg(u8 reg ,u8 data);
void SPI3_readRegs(u8 reg, u8 length, u8 *data);

#endif

//------------------End of File----------------------------
