/*
Camera.c文件
主要功能是从串口4uart4接收串口摄像头的数据，以及向上位机通过usart1发送照片数据
*/

//在上电复位后要等1-2秒摄像头才能接收指令

/******************************************************************************
描述：VC0706摄像头模块驱动函数,支持设备类型： PTC01/PTC06/PTC08

////////////////////
2014/09/11更新
1. 新增修改协议序号，支持RS485
2. 移动侦测拍照

//////////////////
2014-02-21
V1.0版本
摄像头应用实例
CameraDemoApp()

拍照前[可选指令]
设置图片尺寸(默认值：320X240，修改尺寸需复位)
设置图片压缩率(默认值：36)

拍照片的指令顺序：
1.清空图片缓存
2.拍照指令
3.读图片长度指令
4.读图片数据指令

******************************************************************************/
#include "Camera.h"
#include "UART4.h"
#include "delay.h"
#include "USART1DMATX.h"
#include "UART1.h"
#include <stdio.h>
#define CLEAR_FRAME            1   	 //去掉返回图片数据携带的协议头和尾76 00 32 00

#define ID_SERIAL_NUM       1        //序号在数组的所在位置

//复位指令与复位回复
 const unsigned char reset_rsp[] = {0x76,0x00,0x26,0x00};
 const unsigned char reset_cmd[] = {0x56,0x00,0x26,0x00};


//清除图片缓存指令与回复
const unsigned char photoBufCls_cmd [] = {0x56,0x00,0x36,0x01,0x02};
const unsigned char photoBufCls_rsp[] = {0x76,0x00,0x36,0x00,0x00};  	

//拍照指令与回复
const unsigned char start_photo_cmd[] = {0x56,0x00,0x36,0x01,0x00};    
const unsigned char start_photo_rsp[] = {0x76,0x00,0x36,0x00,0x00};   

//读图片长度指令与回复
//图片长度指令回复的前7个字节是固定的，最后2个字节表示图片的长度
//如0xA0,0x00,10进制表示是40960,即图片长度(大小)为40K
const unsigned char read_len_cmd[] = {0x56,0x00,0x34,0x01,0x00};
const unsigned char read_len_rsp[] = {0x76,0x00,0x34,0x00,0x04,0x00,0x00};

//读图片数据指令与回复
//get_photo_cmd前6个字节是固定的，
//第9,10字节是图片的起始地址
//第13,14字节是图片的末尾地址，即本次读取的长度

//如果是一次性读取，第9,10字节的起始地址是0x00,0x00;
//第13,14字节是图片长度的高字节，图片长度的低字节(如0xA0,0x00)

//如果是分次读取，每次读N字节(N必须是8的倍数)长度，
//则起始地址最先从0x00,0x00读取N长度(即N & 0xff00, N & 0x00ff)，
//后几次读的起始地址就是上一次读取数据的末尾地址
const unsigned char get_photo_cmd [] = {0x56,0x00,0x32,0x0C,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF }; 
const unsigned char get_photo_rsp []  = {0x76,0x00,0x32,0x00,0x00};

//设置压缩率指令与回复，最后1个字节为压缩率选项
//范围是：00 - FF
//默认压缩率是36
const unsigned char set_compress_cmd [] = {0x56,0x00,0x31,0x05,0x01,0x01,0x12,0x04,0x36};
const unsigned char compress_rate_rsp [] = {0x76,0x00,0x31,0x00,0x00};

//设置图片尺寸指令与回复
//set_photo_size_cmd最后1个字节的意义
//0x22 - 160X120
//0x11 - 320X240
//0x00 - 640X480
const unsigned char set_photo_size_cmd [] = {0x56,0x00,0x31,0x05,0x04,0x01,0x00,0x19,0x11};
const unsigned char set_photo_size_rsp [] = {0x76,0x00,0x31,0x00,0x00 };

//读取图片尺寸指令与回复
//read_photo_size_rsp最后1个字节的意义
//0x22 - 160X120
//0x11 - 320X240
//0x00 - 640X480
const unsigned char read_photo_size_cmd [] = {0x56,0x00,0x30,0x04,0x04,0x01,0x00,0x19};
const unsigned char read_photo_size_rsp [] = {0x76,0x00,0x30,0x00,0x01,0x00};



//移动侦测指令
//motion_enable_cmd 打开移动侦测
//motion_disable_cmd 关闭移动侦测
const unsigned char motion_enable_cmd [] = {0x56,0x00,0x37,0x01,0x01};
const unsigned char motion_disable_cmd [] = {0x56,0x00,0x37,0x01,0x00};
const unsigned char motion_rsp [] = {0x76,0x00,0x37,0x00,0x00};

//当系统检测到有移动时，自动从串口输出motio_detecte
const unsigned char motion_detecte [] = {0x76,0x00,0x39,0x00,0x00};

//移动侦测灵敏度设置
const unsigned char motion_sensitivity_cmd [] = {0x56,0x00,0x31,0x05,0x01,0x01,0x1A,0x6E,0x03};
const unsigned char motion_sensitivity_rsp [] = {0x76,0x00,0x31,0x00,0x00};

//串口驱动接口函数，移植时需要修改接口函数
void cam_write(const unsigned char *buf,unsigned char len);
uint16_t cam_receiver( unsigned char *buf,uint16_t send_len);



unsigned char camera_init(unsigned char Serialnumber,unsigned char nSetImageSize);
unsigned char send_cmd(const unsigned char *cmd,unsigned char n0,const unsigned char *rev,unsigned char n1);
void SetSerailNumber(unsigned char *DstCmd, const unsigned char *SrcCmd, unsigned char SrcCmdLength,
                     unsigned char *DstRcv, const unsigned char *SrcRcv, unsigned char SrcRcvLength,unsigned char nID);

//摄像头属性设置：复位/图片尺寸大小/图片压缩率
unsigned char send_reset(unsigned char Serialnumber);
unsigned char current_photo_size(unsigned char Serialnumber,unsigned char * nImageSize);
unsigned char send_photo_size(unsigned char Serialnumber,unsigned char nImageSize);
unsigned char send_compress_rate(unsigned char Serialnumber,unsigned char nCompressRate);

//移动侦测控制函数
unsigned char send_motion_sensitivity(unsigned char Serialnumber);
unsigned char send_open_motion(unsigned char Serialnumber);
unsigned char send_close_motion(unsigned char Serialnumber);
unsigned char Motion_Detecte_Idle(unsigned char *pSerialnumber);


//拍照处理函数
unsigned char send_photoBuf_cls(unsigned char Serialnumber);
unsigned char send_start_photo(unsigned char Serialnumber);
uint32_t send_read_len(unsigned char Serialnumber);
unsigned char send_get_photo(uint16_t add,uint16_t read_len,unsigned char *buf,unsigned char Serialnumber);

volatile unsigned char cameraReady = 0;
uint32_t picLen = 0;   //数据长度

//CommandPacket和ResponsePacket用于拷贝只读区的指令及应答到内存
unsigned char CommandPacket[16];
unsigned char ResponsePacket[7];

const nSerialNum SerialNum_Byte;//序列号枚举变量
unsigned char g_SerialNumber = 0;//串口上报移动侦测时保存的当前序列号

void Camera_Routine(void)
{
    CameraPhoto(SERIAL_NUM_0,IMAGE_SIZE_160X120);	
}

void Camera_Int(void)
{
	cameraReady=camera_init(SERIAL_NUM_0, IMAGE_SIZE_160X120);
}

/****************************************************************
函数名：CameraPhoto
函数描述：摄像头应用实例
输入参数：序列号,图片尺寸
返回:成功返回1，失败返回0
******************************************************************/		
unsigned char CameraPhoto(unsigned char Serialnumber,unsigned char nCameraImageSize)
{
    uint16_t cntM = 0, lastBytes = 0, i = 0;
    uint8_t tempCount = 0;
    char *ptr_Report_Buf_C;
    char strToSend[10] = {0};
    //   char buf[50];
    //初始化摄像头
    // cameraReady = camera_init(Serialnumber,nCameraImageSize);
    // if(!cameraReady)
    // {
    //     return 0;
    // }

    //清空图片缓存
    if (!send_photoBuf_cls(Serialnumber))
    {
        return 0;
    }
    // UART1_Put_String((unsigned char *)"send_photoBuf_cls ok\r\n");

    //开始拍照
    if (!send_start_photo(Serialnumber))
    {
        return 0;
    }
    else
    {
        // UART1_Put_String((unsigned char *)"send_start_photo ok\r\n");
        //读取拍照后的图片长度
        picLen = send_read_len(Serialnumber);
        // UART1_Put_String((unsigned char *)"send_read_len ok\r\n");
    }
    if (!picLen)
    {
        return 0;
    }
    else
    {
        cntM = picLen / N_BYTE;
        lastBytes = picLen % N_BYTE;
    }

    sprintf(strToSend, "%04d", picLen);
    USART1WriteDataToBuffer((unsigned char *)strToSend, 4);

    // 分M次，每次读N_BYTE字节
    if (cntM)
    {
        for (i = 0; i < cntM; i++)
        {            
            memset(Report_Buf, 0, sizeof(Report_Buf));
            // memset(Report_Buf_C, 0, N_BYTE * 2 + 1);

            //按图片长度读取数据
            if (!send_get_photo(i * N_BYTE,
                                N_BYTE,
                                Report_Buf,
                                Serialnumber))
            {
                return 0;
            }
            else
            {
                //此分支可将图片数据输出到指定的串口
                //如接口函数，将图片数据写入到串口1
                char Report_Buf_C[N_BYTE * 2 +1] = {0};
                ptr_Report_Buf_C = Report_Buf_C;
                for (tempCount = 0; tempCount < N_BYTE;tempCount++)
                {
                    sprintf(ptr_Report_Buf_C, "%02X", Report_Buf[tempCount]);
                    ptr_Report_Buf_C += 2 ;
                }
                USART1WriteDataToBuffer((unsigned char *)Report_Buf_C, (unsigned char)N_BYTE*2);
            // USART1WriteDataToBuffer("\r\n", 2);
                // USART1WriteDataToBuffer(Report_Buf, (unsigned char)N_BYTE);
            // USART1WriteDataToBuffer("\r\n", 2);
            }

            //   delay_ms(100);
        }
    }
    //剩余图片长度
    if (lastBytes)
    {

        memset(Report_Buf, 0, sizeof(Report_Buf));
        // memset(Report_Buf_C, 0, N_BYTE * 2 + 1);

        //读取剩余长度
        if (!send_get_photo(i * N_BYTE,
                            lastBytes,
                            Report_Buf,
                            Serialnumber))
        {
            return 0;
        }
        else
        {
            char Report_Buf_C[N_BYTE * 2 +1] = {0};
            ptr_Report_Buf_C = Report_Buf_C;
            for (tempCount = 0; tempCount < lastBytes;tempCount++)
            {
                sprintf(ptr_Report_Buf_C, "%02X", Report_Buf[tempCount]);
                ptr_Report_Buf_C += 2 ;
            }
            USART1WriteDataToBuffer((unsigned char *)Report_Buf_C, (unsigned char)lastBytes*2);
            // USART1WriteDataToBuffer("\r\n", 2);
            // USART1WriteDataToBuffer(Report_Buf, (unsigned char)lastBytes);
            // USART1WriteDataToBuffer("\r\n", 2);
        }
        //  delay_ms(100);
    }

    return 1;
}


/****************************************************************
函数名：SetSerailNumber
函数描述: 修改协议中的序号
输入参数：目标指令缓存首地址，源指令首地址，源指令长度，
          目标应答缓存首地址，源应答首地址，源应答长度，需要修改的
          序号值
返回:无
******************************************************************/		
void SetSerailNumber(unsigned char *DstCmd, const unsigned char *SrcCmd, unsigned char SrcCmdLength,
                     unsigned char *DstRsp, const unsigned char *SrcRsp, unsigned char SrcRspLength,unsigned char nID)
{
    memset(&CommandPacket,0,sizeof(CommandPacket));
    memset(&ResponsePacket,0,sizeof(ResponsePacket));
    
    memcpy(DstCmd,SrcCmd,SrcCmdLength);
    memcpy(DstRsp,SrcRsp,SrcRspLength);
    
    DstCmd[ID_SERIAL_NUM] = nID & 0xFF;
    DstRsp[ID_SERIAL_NUM] = nID & 0xFF;

}

/****************************************************************
函数名：cam_write
函数描述: 接口函数，写入控制摄像头的串口
输入参数：数据的首地址，长度
返回:无
******************************************************************/		
void cam_write(const unsigned char *buf,unsigned char len)
{ 
    //写串口驱动函数
    UART4Write((unsigned char *)buf,len);
    
}

/****************************************************************
函数名：cam_receiver
函数描述：接口函数，读取控制摄像头的串口
输出参数：接收数据的地址，长度
返回:接收到数据个数
******************************************************************/		
uint16_t cam_receiver(unsigned char *buf,uint16_t send_len)
{ 
    uint16_t i = 0;
    
    i = UART4_Receiver_buf(buf,send_len,Z_TIME);

    return i;
}

/****************************************************************
函数名：camera_init
函数描述：摄像头初始化
输入参数：序列号，需要设置的图片尺寸
返回:初始化成功返回1，初始化失败返回0
******************************************************************/		
unsigned char camera_init(unsigned char Serialnumber,unsigned char nSetImageSize)
{    
    unsigned char CurrentImageSize = 0xFF;
    unsigned char CurrentCompressRate = COMPRESS_RATE_60;
    UART1_Put_String((unsigned char *)"start init\r\n");
    //读取当前的图片尺寸到currentImageSize
    if ( !current_photo_size(Serialnumber,&CurrentImageSize))
    {
        return 0;
    }
    UART1_Put_String((unsigned char *)"read photo size ok\r\n");
    
    //判断是否需要修改图片尺寸
    if(nSetImageSize != CurrentImageSize)
    {
        //设置图片尺寸，设置后复位生效，该项设置后会永久保存
        if ( !send_photo_size(Serialnumber,nSetImageSize))
        {
            return 0;
        }
        else
        {
    UART1_Put_String((unsigned char *)"write photo size ok\r\n");
            //复位生效
            if ( !send_reset(Serialnumber))
            {
                return 0;
            }
    UART1_Put_String((unsigned char *)"reset ok\r\n");
            delay_ms(1000);
            CurrentImageSize = nSetImageSize;
        }
        
    }
    
    //给不同图片尺寸设置适当的图片压缩率
    if(nSetImageSize == CurrentImageSize)
    {
        switch(CurrentImageSize)
        {
            case IMAGE_SIZE_160X120:
                 CurrentCompressRate = COMPRESS_RATE_60;
                 break;
            case IMAGE_SIZE_320X240:
                 CurrentCompressRate = COMPRESS_RATE_60;
                 break;
            case IMAGE_SIZE_640X480:
                 CurrentCompressRate = COMPRESS_RATE_60;
                 break;
            default:
                break;
        }
    }
    //设置图片压缩率，该项不保存，每次上电后需重新设置
    if ( !send_compress_rate(Serialnumber,CurrentCompressRate))
    {
        return 0;
    }

    //这里要注意,设置压缩率后要延时
    delay_ms(100);

    return 1;
    
}

/****************************************************************
 函数名：send_cmd
 函数描述：发送指令并识别指令返回
 输入参数：指令的首地址，指令的长度，匹配指令的首地址，需验证的个数
 返回：成功返回1,失败返回0
******************************************************************/	
unsigned char send_cmd( const unsigned char *cmd,unsigned char n0,const unsigned char *rev,unsigned char n1)
{
    unsigned char  i;
    unsigned char  tmp[5] = {0x00,0x00,0x00,0x00,0x00};

    cam_write(cmd, n0);

    if ( !cam_receiver(tmp,5) ) 
    {
        return 0;
    }
    // UART1_Put_String((unsigned char *)"recv ok\r\n");
    //检验数据
    for (i = 0; i < n1; i++)
    {  
        if (tmp[i] != rev[i]) 
        {
            return 0;
        }
    }
    // UART1_Put_String((unsigned char *)"check ok\r\n");
		
    return 1;

}


/****************************************************************
函数名：current_photo_size
函数描述:读取当前设置的图片尺寸
输入参数：Serialnumber序列号，nImageSize传递图片尺寸的引用变量
返回:成功返回1,失败返回0
******************************************************************/	
unsigned char current_photo_size(unsigned char Serialnumber,unsigned char * nImageSize)
{  
    unsigned char  i;
    unsigned char  tmp[6] = {0x00,0x00,0x00,0x00,0x00,0x00};

    SetSerailNumber( CommandPacket,
                     read_photo_size_cmd,
                     sizeof(read_photo_size_cmd),
                     ResponsePacket,
                     read_photo_size_rsp,
                     sizeof(read_photo_size_rsp),
                     Serialnumber );
    
    cam_write(CommandPacket, sizeof(read_photo_size_cmd));

    if ( !cam_receiver(tmp,6) ) 
    {
        return 0;
    }
    // UART1_Put_String((unsigned char *)"receiv ok\r\n");
    //检验数据,对比前5个字节
    for (i = 0; i < 5; i++)
    {  
        if (tmp[i] != ResponsePacket[i]) 
        {
            return 0;
        }
    }
    
    //最后一个字节表示当前的图片大小
    *nImageSize = tmp[5];
    return 1;
}


/****************************************************************
函数名：send_photo_size
函数描述：设置拍照的图片尺寸（可选择：160X120,320X240,640X480）
输入参数：序列号，需要设置的图片尺寸
返回:成功返回1,失败返回0
******************************************************************/	
unsigned char send_photo_size(unsigned char Serialnumber,unsigned char nImageSize)
{  
    unsigned char  i;
    
    SetSerailNumber( CommandPacket,
                     set_photo_size_cmd,
                     sizeof(set_photo_size_cmd),
                     ResponsePacket,
                     set_photo_size_rsp,
                     sizeof(set_photo_size_rsp),
                     Serialnumber );
    
    CommandPacket [sizeof(set_photo_size_cmd) - 1] = nImageSize;
    
    i = send_cmd( CommandPacket,
                  sizeof(set_photo_size_cmd),
                  ResponsePacket,
                  sizeof(set_photo_size_rsp) );
    return i;
}


/****************************************************************
函数名：send_reset
函数描述：发送复位指令复位后要延时1-2秒
输入参数：序列号
返回:成功返回1 失败返回0
******************************************************************/		
unsigned char send_reset(unsigned char Serialnumber)
{  
    unsigned char i;
    //复制命令与应答，修改序号
    SetSerailNumber( CommandPacket,
                     reset_cmd,
                     sizeof(reset_cmd),
                     ResponsePacket,
                     reset_rsp,
                     sizeof(reset_rsp),
                     Serialnumber );
    
    i = send_cmd( CommandPacket,
                  sizeof(reset_cmd),
                  ResponsePacket,
                  sizeof(reset_rsp) );
    
    return i;

}

/****************************************************************
函数名：send_stop_photo
函数描述：清空图片缓存
输入参数：序列号
返回:成功返回1,失败返回0
******************************************************************/		 
unsigned char send_photoBuf_cls(unsigned char Serialnumber)
{ 
    unsigned char i;
    
    SetSerailNumber( CommandPacket,
                     photoBufCls_cmd,
                     sizeof(photoBufCls_cmd),
                     ResponsePacket,
                     photoBufCls_rsp,
                     sizeof(photoBufCls_rsp),
                     Serialnumber );
    
    i = send_cmd( CommandPacket,
                  sizeof(photoBufCls_cmd),
                  ResponsePacket,
                  sizeof(photoBufCls_rsp) );
    return i;
}  


/****************************************************************
函数名：send_compress_rate
函数描述：发送设置图片压缩率
输入参数：序列号
返回:成功返回1,失败返回0
******************************************************************/		 
unsigned char send_compress_rate(unsigned char Serialnumber,unsigned char nCompressRate)
{
    unsigned char i;
    
    SetSerailNumber( CommandPacket,
                     set_compress_cmd,
                     sizeof(set_compress_cmd),
                     ResponsePacket,
                     compress_rate_rsp,
                     sizeof(compress_rate_rsp),
                     Serialnumber );
    
    if(nCompressRate > 0x36)
    {
        //最后一个字节表示压缩率
        CommandPacket [sizeof(set_compress_cmd) - 1] = nCompressRate;
    }
    
    i = send_cmd( CommandPacket,
                  sizeof(set_compress_cmd),
                  ResponsePacket,
                  sizeof(compress_rate_rsp) );
    return i;
}


/****************************************************************
函数名：send_start_photo
函数描述：发送开始拍照的指令
输入参数：序列号
返回:识别成功返回1 失败返回0
******************************************************************/		
unsigned char send_start_photo(unsigned char Serialnumber)
{
    unsigned char i;
    
    SetSerailNumber( CommandPacket,
                     start_photo_cmd,
                     sizeof(start_photo_cmd),
                     ResponsePacket,
                     start_photo_rsp,
                     sizeof(start_photo_rsp),
                     Serialnumber );
    
    i = send_cmd( CommandPacket,
                  sizeof(start_photo_cmd),
                  ResponsePacket,
                  sizeof(start_photo_rsp) );
    return i;
}	  


/****************************************************************
函数名：send_read_len
函数描述：读取拍照后的图片长度，即图片占用空间大小
输入参数：序列号
返回:图片的长度
******************************************************************/	
uint32_t send_read_len(unsigned char Serialnumber)
{
    unsigned char i;
    uint32_t len;
    unsigned char tmp[9];	
    
    SetSerailNumber( CommandPacket,
                     read_len_cmd,
                     sizeof(read_len_cmd),
                     ResponsePacket,
                     read_len_rsp,
                     sizeof(read_len_rsp),
                     Serialnumber );
    
    //发送读图片长度指令
    cam_write(CommandPacket, 5);

    if ( !cam_receiver(tmp,9)) 
    {
        return 0;
    }

    //检验数据
    for (i = 0; i < 7; i++)
    {
        if ( tmp[i] != ResponsePacket[i]) 
        {
            return 0;
        }
    }
    
    len = (uint32_t)tmp[7] << 8;//高字节
    len |= tmp[8];//低字节
    
    return len;
}


/****************************************************************
函数名：send_get_photo
函数描述：读取图片数据
输入参数：读图片起始地址StaAdd, 
          读取的长度readLen ，
          接收数据的缓冲区buf
          序列号
返回:成功返回1，失败返回0
FF D8 ... FF D9 是JPG的图片格式

1.一次性读取的回复格式：76 00 32 00 00 FF D8 ... FF D9 76 00 32 00 00

2.分次读取，每次读N字节,循环使用读取图片数据指令读取M次或者(M + 1)次读取完毕：
如第一次执行后回复格式
76 00 32 00 <FF D8 ... N> 76 00 32 00
下次执行读取指令时，起始地址需要偏移N字节，即上一次的末尾地址，回复格式
76 00 32 00 <... N> 76 00 32 00
......
76 00 32 00 <... FF D9> 76 00 32 00 //lastBytes <= N

Length = N * M 或 Length = N * M + lastBytes

******************************************************************/	
unsigned char send_get_photo(uint16_t staAdd,uint16_t readLen,unsigned char *buf,unsigned char Serialnumber)
{
    unsigned char i = 0;
    unsigned char *ptr = NULL;
    
    
    SetSerailNumber( CommandPacket,
                     get_photo_cmd,
                     sizeof(get_photo_cmd),
                     ResponsePacket,
                     get_photo_rsp,
                     sizeof(get_photo_rsp),
                     Serialnumber );

    //装入起始地址高低字节
    CommandPacket[8] = (staAdd >> 8) & 0xff;
    CommandPacket[9] = staAdd & 0xff;
    //装入末尾地址高低字节
    CommandPacket[12] = (readLen >> 8) & 0xff;
    CommandPacket[13] = readLen & 0xff;

    
    //执行指令
    cam_write(CommandPacket,16);
    
    //等待图片数据存储到buf，超时或无数据回复则返回0
    if ( !cam_receiver(buf,readLen + 10))
    {
        return 0;
    }
    
    //检验帧头76 00 32 00 00
    for (i = 0; i < 5; i++)
    {
        if ( buf[i] != ResponsePacket[i] )
        {
            return 0;
        }
    }

    //检验帧尾76 00 32 00 00
    for (i = 0; i < 5; i++)
    {
        if ( buf[i + 5 + readLen] != ResponsePacket[i] )
        {
					//usart_SendString("\r\r no data!\r\n");
            return 0;
        }
    }
    	
   // usart_SendString("\r\r no data!\r\n");
    //宏开关选择丢弃/保留 帧头帧尾76 00 32 00 00
    #if CLEAR_FRAME
//    memcpy(buf,buf + 5,read_len);
    ptr = buf;
    
    for (; readLen > 0; ++ptr)
    {
        *(ptr) = *(ptr + 5);
        readLen--;
    }
    #endif
    
    return 1;
}

/****************************************************************
函数名：send_open_motion
函数描述：发送打开移动侦测指令
输入参数：序列号
返回:识别成功返回1 失败返回0
******************************************************************/		
unsigned char send_motion_sensitivity(unsigned char Serialnumber)
{
    unsigned char i;
    
    SetSerailNumber( CommandPacket,
                     motion_sensitivity_cmd,
                     sizeof(motion_sensitivity_cmd),
                     ResponsePacket,
                     motion_sensitivity_rsp,
                     sizeof(motion_sensitivity_rsp),
                     Serialnumber );
    
    i = send_cmd( CommandPacket,
                  sizeof(motion_sensitivity_cmd),
                  ResponsePacket,
                  sizeof(motion_sensitivity_rsp) );
    return i;
}

/****************************************************************
函数名：send_open_motion
函数描述：发送打开移动侦测指令
输入参数：序列号
返回:识别成功返回1 失败返回0
******************************************************************/		
unsigned char send_open_motion(unsigned char Serialnumber)
{
    unsigned char i;
    
    SetSerailNumber( CommandPacket,
                     motion_enable_cmd,
                     sizeof(motion_enable_cmd),
                     ResponsePacket,
                     motion_rsp,
                     sizeof(motion_rsp),
                     Serialnumber );
    
    i = send_cmd( CommandPacket,
                  sizeof(motion_enable_cmd),
                  ResponsePacket,
                  sizeof(motion_rsp) );
    return i;
}

/****************************************************************
函数名：send_close_motion
函数描述：发送关闭移动侦测指令
输入参数：序列号
返回:识别成功返回1 失败返回0
******************************************************************/		
unsigned char send_close_motion(unsigned char Serialnumber)
{
    unsigned char i;
    
    SetSerailNumber( CommandPacket,
                     motion_disable_cmd,
                     sizeof(motion_disable_cmd),
                     ResponsePacket,
                     motion_rsp,
                     sizeof(motion_rsp),
                     Serialnumber );
    
    i = send_cmd( CommandPacket,
                  sizeof(motion_disable_cmd),
                  ResponsePacket,
                  sizeof(motion_rsp) );
    return i;
}

/****************************************************************
函数名：Motion_Detecte_Idle
函数描述: 等待移动侦测事件,该函数可在RS485同时接多个摄像头时，传递
          当前是第几个序列号上报移动侦测
输入参数：传递一个指针变量
返回:成功返回1 失败返回0
******************************************************************/		
unsigned char Motion_Detecte_Idle(unsigned char *pSerialnumber)
{
    unsigned char  tmp[5] = {0x00,0x00,0x00,0x00,0x00};
    
    if ( !cam_receiver(tmp,5) ) 
    {
        return 0;
    }
    
    //检验数据
    //全部有5个数据，只校验4个，其中数组下标为1是序列号
    if(!(tmp[0] == motion_detecte[0] && 
         tmp[2] == motion_detecte[2] &&
         tmp[3] == motion_detecte[3] &&
         tmp[4] == motion_detecte[4] ))
    {
        return 0;
    }
    
    //取出序列号
    *pSerialnumber = tmp[1];
    return 1;

}

