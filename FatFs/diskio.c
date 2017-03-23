/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "ff.h"
#include "diskio.h"
#include "SD_card.h"


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	u8 res=0;
	 
	res = SD_Initialize();
	if(res) //STM32 SPI的bug,在sd卡操作失败的时候如果不执行下面的语句,可能导致SPI读写异常
			{
				SD_SPI_SpeedLow();
				SD_SPI_ReadWriteByte(0xff);//提供额外的8个时钟
				SD_SPI_SpeedHigh();
			}
	if(res)return  STA_NOINIT;
	else return 0; //初始化成功									  
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{ 
    //检查SD卡是否插入
    if(!SD_DET())
    {
        return STA_NODISK;
    }
    return 0;

}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
	u8 res=0;
    if(!SD_DET())
    {
        return RES_NOTRDY;  //没有检测到SD卡，报NOT READY错误
    }
	
	res = SD_ReadDisk(buff,sector,count);	 
	if(res)
			{
				SD_SPI_SpeedLow();
				SD_SPI_ReadWriteByte(0xff);//提供额外的8个时钟
				SD_SPI_SpeedHigh();
			}

  	//处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;                                               

}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	u8 res=0;
	if (!count)return RES_ERROR;//count不能等于0，否则返回参数错误
	res=SD_WriteDisk((u8*)buff,sector,count);
	//处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    DRESULT res;


    if (drv)
    {    
        return RES_PARERR;  //仅支持单磁盘操作，否则返回参数错误
    }
    
    //FATFS目前版本仅需处理CTRL_SYNC，GET_SECTOR_COUNT，GET_BLOCK_SIZ三个命令
    switch(ctrl)
    {
    case CTRL_SYNC:
	         SD_CS_L();
			 if(SD_WaitReady()==0)res = RES_OK; 
			  	else res = RES_ERROR;	  
			 SD_CS_H();
	         break;
    case GET_SECTOR_SIZE:
		        *(WORD*)buff = 512;
		        res = RES_OK;
		        break;	 
	case GET_BLOCK_SIZE:
		        *(WORD*)buff = 8;
		        res = RES_OK;
		        break;
    case GET_SECTOR_COUNT:
	        *(DWORD*)buff = SD_GetSectorCount();
	        res = RES_OK;
	        break;
    default:
        res = RES_PARERR;
        break;
    }

    return res;
}


/*-----------------------------------------------------------------------*/
/* User defined function to give a current time to fatfs module          */
/* 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
/* 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{
    DWORD date = 0;
    return date;
}
							




