#ifndef  __CAMERA_H__
#define  __CAMERA_H__

#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"


// typedef unsigned char u8;
// typedef unsigned short int u16;
// typedef unsigned int u32;


//用户自定义宏变量
#define N_BYTE  64//13568       //每次读取N_BYTE字节，N_BYTE必须是8的倍数
                 
#define IMAGE_SIZE_160X120     0x22
#define IMAGE_SIZE_320X240     0x11
#define IMAGE_SIZE_640X480     0x00


#define COMPRESS_RATE_36       0x36   //该压缩率是默认压缩率，160x120和320x240可用此压缩率

#define COMPRESS_RATE_60       0x60   //640X480尺寸，默认压缩率36会占用45K左右的空间
                                      //选择60压缩率可将45K压缩到20K左右

typedef enum
{
    SERIAL_NUM_0 = 0x00,
    SERIAL_NUM_1,
    SERIAL_NUM_2,
    SERIAL_NUM_3,
    SERIAL_NUM_5,
    SERIAL_NUM_6,
    SERIAL_NUM_7,
    SERIAL_NUM_8,
    SERIAL_NUM_9,
    SERIAL_NUM_10
}nSerialNum;
extern const nSerialNum SerialNum_Byte;

extern unsigned char g_SerialNumber;
extern volatile unsigned char cameraReady;
extern u32 picLen;//图片长度

unsigned char CameraPhoto(unsigned char Serialnumber, unsigned char nCameraImageSize);
void Camera_Routine(void);
void Camera_Int(void);

#endif
