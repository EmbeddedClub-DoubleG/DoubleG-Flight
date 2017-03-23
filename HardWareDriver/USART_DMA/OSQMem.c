
/******************** 版权所有 北京工业大学焊接研究所 潘健 ********************
* 文件名             : OSQMem.c
* 作者               : 潘健
* 版本               : V1.2
* 日期               : 1/13/2010
* 描述               : 内存的管理函数
* 功能               ：内存的管理函数				   				   	
*******************************************************************************/

#include "stdio.h"
#include "OSQMem.h"				
OSMEMTcb OSMemTcb[OS_MEM_MAX];						//内存块管理区定义，最多允许OS_MEM_MAX个内存区

u8 OSUSART1MemQ[OS_MEM_USART1_MAX];  				//空白内存块,OS_MEM_USART1_MAX表示内存块区中的最大数量



/*******************************************************************************
* 文件名	  	 : OSMemInit
* 描述	         : 空白内存块的初始化函数，
* 移植步骤		 : 无
* 输入           : ptr空白连续内存区的指针，count空白连续内存区内的数量，
* 输出           : 无
* 返回           : 内存块的首地址
*******************************************************************************/
OSMEMTcb *OSMemInit(u8 *ptr,u16 count)
{
	u16 i;
	for(i=0;i<count;i++)
	{
		ptr[i]=0;
	}
	return 	(OSMEMTcb*)*ptr;
}

/*******************************************************************************
* 文件名	  	 : OSMemCreate
* 描述	         : 建立一个内存管理区（具体请参考UCOS的内存管理结构）
* 移植步骤		 : 无
* 输入           : ptr空白连续内存区的指针， blksize每个内存块有多少个字节，
				   nblks多少个内存块，
* 输出           : 无
* 返回           : 内存块的首地址
*******************************************************************************/		
OSMEMTcb *OSMemCreate(u8 *ptr,u8 blksize,u8 nblks,u8 *err)
{
	u8 *link=(ptr+blksize);
	void **plink=(void **)ptr;
	u16 i=0;
	u16 j=0;
	/*首先查找空白的内存管理区*/
	while((OSMemTcb[i].OSMemFreeList!=(u8*)0)&&(i<=OS_MEM_MAX))		
	{														
		i++;
	}
	if(i>=OS_MEM_MAX){*err=0xff;return (OSMEMTcb *)0;}		//内存块管理区分配不到，返回空指针						
	OSMemTcb[i].OSMemFreeList=ptr;							//内存块管理区的内存的指针
	OSMemTcb[i].OSMemBlkSize=blksize;						//内存块管理区的每个内存块的字节
	OSMemTcb[i].OSMemNBlks=nblks;							//内存块管理区的内存块的总数量
	OSMemTcb[i].OSMemFreeNBlks=nblks;						//内存块管理区的空白内存块的数量
	j=i;
	OSMemInit(ptr,blksize*nblks);							//将内存块内的数据初始化
	for(i=0;i<nblks-1;i++)									//
	{
		*plink=(void *)(link);								//该内存块的地址存放的是第二片内存区的首地址
		plink=(void **)(link);								//将二维指针定位到框的首位		
		link+=blksize;										//一维指针重新定位
	}
//	*plink=(void *)0;										//最后一块内存区的下一个地址块的数据为0
	return (OSMEMTcb *)(OSMemTcb+j); 										//返回内存块管理区的首地址
}	
/*******************************************************************************
* 文件名	  	 : OSMemGet
* 描述	         : 从一个内存管理区获取一个内存块
* 移植步骤		 : 无
* 输入           : ptr内存管理区的指针
* 输出           : 无
* 返回           : 获取的空白内存块的首地址
*******************************************************************************/
u8 *OSMemGet(OSMEMTcb *ptr,u8 *err)
{
	void *tcb;
	u8 *index;
	tcb=(*ptr).OSMemFreeList;
	if((*ptr).OSMemFreeNBlks==0){*err=0xff;return (void *)0;}//如果空白内存块的数量为返回,若正确返回，收到的数据应该是0
	(*ptr).OSMemFreeNBlks--;								//空白内存块块数量减一
	if((*ptr).OSMemFreeNBlks!=0)(*ptr).OSMemFreeList=*(void **)tcb;	//强制类型转换
	index=(u8 *)tcb;
	index+=4;
	*err=0;
	return index;	 										//返回获取的空白内存块的首地址
}
/*******************************************************************************
* 文件名	  	 : SMemDelete
* 描述	         : 从一个内存管理区删除一个内存块
* 移植步骤		 : 无
* 输入           : ptr内存管理区的指针
* 输出           : 无
* 返回           : 如果要删除的内存块是一个空指针，则返回0xff，若能够删除，返回0
*******************************************************************************/
u8 OSMemDelete(OSMEMTcb *ptr,u8 *index)
{
	void *tcb;
	if(index==(void*)0)return 0xff;					   		//如果要删除的内存块是一个空指针，则返回0xff
	index-=4;
	tcb=(void *)index;	
	*(void **)tcb=(*ptr).OSMemFreeList;						//将OSMemFreeList重新指向这个已经变成空白了的指针
	(*ptr).OSMemFreeList=tcb;								//将这个空白的指针的下一个指针指向一个原先的OSMemFreeList												//
	(*ptr).OSMemFreeNBlks++;								//空白内存块数量加1
	return 0;	 		
}

//------------------End of File----------------------------
