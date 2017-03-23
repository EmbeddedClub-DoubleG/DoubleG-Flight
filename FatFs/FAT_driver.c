#include "FAT_driver.h"

FATFS fs;  		//逻辑磁盘工作区.	 
FIL file;	  		//文件1
FIL ftemp;	  		//文件2.
UINT br,bw;			//读写变量
FILINFO fileinfo;	//文件信息
DIR dir;  			//目录

u8 fatbuf[512];			//SD卡数据缓存区


void FAT_Initial(void){
	u16 try = 0;
	while(SD_Initialize()){
		if(++try > 20)return;
	}
	f_mount(0,&fs);
}

//打开路径下的文件
//path:路径+文件名
//mode:打开模式
//返回值:执行结果
u8 mf_open(u8 *path,u8 mode)
{
	u8 res;	 
	res=f_open(&file,(TCHAR*)path,mode);//打开文件夹
	return res;
} 
//关闭文件
//返回值:执行结果
u8 mf_close(void)
{
	f_close(&file);
	return 0;
}
//读出数据
//len:读出的长度
//返回值:执行结果
u8 mf_read(u16 len)
{
	u16 i;
	u8 res=0;
	u16 tlen=0;
	for(i=0;i<len/512;i++)
	{
		res=f_read(&file,fatbuf,512,&br);
		if(res)
		{
			//printf("Read Error:%d\r\n",res);
			break;
		}else
		{
			tlen+=br;
		//	for(t=0;t<br;t++)printf("%c",fatbuf[t]); 
		}
	}
	if(len%512)
	{
		res=f_read(&file,fatbuf,len%512,&br);
		if(res)	//读数据出错了
		{  
		}else
		{
			tlen+=br; 
		}	 
	} 
	return res;
}
//写入数据
//dat:数据缓存区
//len:写入长度
//返回值:执行结果
u8 mf_write(u8*dat,u16 len)
{			    
	u8 res;	   					   	 
	res = f_write(&file,dat,len,&bw);
	//f_sync(&file);
	return res;
}

//文件读写指针偏移
//offset:相对首地址的偏移量
//返回值:执行结果.
u8 mf_lseek(u32 offset)
{
	return f_lseek(&file,offset);
}
