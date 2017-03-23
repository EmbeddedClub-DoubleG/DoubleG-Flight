#ifndef __AT45DB_H
#define __AT45DB_H

#include "stm32f4xx.h"

#define AT45DB_BUFFER_1_WRITE                 0x84 /* 写入第一缓冲区 */
#define AT45DB_BUFFER_2_WRITE                 0x87 /* 写入第二缓冲区 */
#define AT45DB_BUFFER_1_WRITE_FLASH           0x82 /* 写入第一缓冲区 */
#define AT45DB_BUFFER_2_WRITE_FLASH           0x85 /* 写入第二缓冲区 */

#define AT45DB_BUFFER_1_READ                  0xD4 /* 读取第一缓冲区 */
#define AT45DB_BUFFER_2_READ                  0xD6 /* 读取第二缓冲区 */
#define AT45DB_B1_TO_MM_PAGE_PROG_WITH_ERASE  0x83 /* 将第一缓冲区的数据写入主存储器（擦除模式）*/
#define AT45DB_B2_TO_MM_PAGE_PROG_WITH_ERASE  0x86 /* 将第二缓冲区的数据写入主存储器（擦除模式）*/
#define AT45DB_MM_PAGE_TO_B1_XFER             0x53 /* 将主存储器的指定页数据加载到第一缓冲区    */
#define AT45DB_MM_PAGE_TO_B2_XFER             0x55 /* 将主存储器的指定页数据加载到第二缓冲区    */
#define AT45DB_PAGE_ERASE                     0x81 /* 页删除（每页512/528字节） */
#define AT45DB_SECTOR_ERASE                   0x7C /* 扇区擦除（每扇区128K字节）*/
#define AT45DB_READ_STATE_REGISTER            0xD7 /* 读取状态寄存器 */
#define AT45DB_BLACK_ERASE       			  0x50 /* 块删除(每块4KByte)*/
#define AT45DB_MM_PAGE_READ       		      0xD2 /* 直接读主存储器的内存页*/
#define AT45DB_Continuous_Read                0x0B 


#define AT45DB_16HZ_Block_Number   						64  //占用256K字节存储空间,那么要占用256/4=64个Block

typedef enum {FLASH_BUFFER1=0, FLASH_BUFFER2} FLASH_BUFFER_NUM;


#define FLASH_PAGE_COUNT   4096 // 0..4095
#define FLASH_PAGE_SIZE    512
#define FLASH_SIZE_KBYTE   2112

#define AT45db161_SPI_SELECT()  GPIO_ResetBits(GPIOB, GPIO_Pin_12)       /* AT45db161 CS = L */ 
#define AT45db161_SPI_DESELECT()  GPIO_SetBits(GPIOB, GPIO_Pin_12)       /* AT45db161 CS = H */

// set Page (12Bit) and Byte Address (10Bit)
#define ADR_P_B_1(page)    (unsigned char)(page>>6)    // 2 res.bits + 6 bits of 12-bit page-address
#define ADR_P_B_2(page, address) (unsigned char)(page<<2)|(address>>8) // 6 bit page-address and 2bit Byte-address
#define ADR_P_B_3(address)      (unsigned char)address     // 8 bit Byte-address

// set Byte Address (10Bit)
#define ADR_B_1(address)      (unsigned char)(address>>8)    // 6 don't care + Bit8,9 of address (2bit address)
#define ADR_B_2(address)   (unsigned char)(address)    // bit7 .. Bit0 of address (8bit address)

// set Page Address (12Bit)
#define ADR_P_1(page)    (unsigned char)(page>>6)    // 2 res.bits + 6 bits of 12-bit page-address
#define ADR_P_2(page)    ((unsigned char)(page))<<2    // 6 bit of 12-bit page-address + 2 don't care

extern volatile struct data_map Config;

void SPI2_SetSpeed(uint16_t SpeedSet);
uint8_t SPI2_ReadWrite_Byte(uint8_t byte);

void AT45db161_SPI_Configuration(void);
uint8_t AT45DB_Check(void);
void AT45DB_Write_Bytes(uint32_t add,uint8_t *pdata, uint16_t len);
void AT45DB_Read_Bytes(uint32_t add,uint8_t *pdata, uint16_t len);

void AT45DB_Write_float(uint32_t add, float wvalue);
float AT45DB_Read_float(uint32_t add);
void AT45DB_Write_int16(uint32_t add, int16_t wvalue);
int16_t AT45DB_Read_int16(uint32_t add);

void AT45DB_Read_config(void);
void AT45DB_Write_config(void);

#endif

//------------------End of File----------------------------
