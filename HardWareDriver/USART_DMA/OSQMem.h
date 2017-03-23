
/******************** 版权所有 北京工业大学焊接研究所 潘健 ********************
* 文件名             : OSQMem.h
* 作者               : 潘健
* 版本               : V1.2
* 日期               : 1/13/2010
* 描述               : 内存的管理函数
* 功能               ：内存的管理函数				   				   	
*******************************************************************************/
#ifndef _OSQMEM_H
#define _OSQMEM_H

#include "stm32f4xx.h"

#define 	OS_MEM_MAX 				8	   			//最多允许的内存块管理区
#define 	OS_MEM_USART1_MAX 		1024			//发送缓冲区的内存大小
#define 	OS_MEM_USART1_BLK 		32				//每一个块的长度

typedef struct OSMEMTCB{
	void 		*OSMemFreeList;
	u8 			OSMemBlkSize;
	u8 			OSMemNBlks;
	u8 			OSMemFreeNBlks;
}OSMEMTcb;

OSMEMTcb *OSMemInit(u8 *ptr,u16 count);
OSMEMTcb *OSMemCreate(u8 *ptr,u8 blksize,u8 nblks,u8 *err);
u8 *OSMemGet(OSMEMTcb *ptr,u8 *err);
u8 OSMemDelete(OSMEMTcb *ptr,u8 *index);
#endif

//------------------End of File----------------------------
