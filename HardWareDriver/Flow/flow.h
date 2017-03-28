#ifndef __FLOW_H
#define __FLOW_H
#include "common.h"
//器件地址
#define FLOW_ADDR  0x42
//存储器宏定义，如果是2字节1个数据，低字节地低位，高字节高位

//未积分的数据
//记录总的创建的IIC的帧数
#define FRAME_COUNT_SUM     0x00                    //uint16_t counts created I2C frames [frames]
//X轴最新一帧所有像素移动和*10
#define PIXEL_FLOW_X_SUM    0x02                    //int16_t latest x flow measurement in pixels*10 [pixels]
//Y轴最新一帧所有像素移动和*10
#define PIXEL_FLOW_Y_SUM    0x04                    //int16_t latest y flow measurement in pixels*10 [pixels]
//X轴速度
#define FLOW_COMP_M_X       0x06                    //int16_t x velocity*1000 [meters/sec]
//Y轴速度
#define FLOW_COMP_M_Y       0x08                    //int16_t y velocity*1000 [meters/sec]
//光流图像质量
#define QUAL                0x0a                    //int16_t Optical flow quality / confidence [0: bad, 255: maximum quality]
//X轴陀螺仪速度
#define GYRO_X_RATE         0x0c                    //int16_t latest gyro x rate [rad/sec]
//Y轴陀螺仪速度
#define GYRO_Y_RATE         0x0e                    //int16_t latest gyro y rate [rad/sec]
//Z轴陀螺仪速度
#define GYRO_Z_RATE         0x10                    //int16_t latest gyro z rate [rad/sec]
//陀螺仪数据范围
#define GYRO_RANGE          0x12                    //uint8_t gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
//超声波数据上次更新到现在的时间间隔
#define SONAR_TIMESTAMP1     0x13                    //uint8_t time since last sonar update [milliseconds] 
//地面距离  正值：已知距离   负值：未知距离
#define GROUND_DISTANCE1    0x14                    //int16_t Ground distance in meters*1000 [meters]. Positive value: distance known. 
                                                            //Negative value: Unknown distance
//积分后的数据
//上次读取数据后，数据更新了多少次
#define FRAME_COUNT_SINCE_LAST_READOUT  0x16        //uint16_t number of flow measurements since last I2C readout [frames]
//上次读取IIC数据后，X轴速度积分后所得值
#define PIXEL_FLOW_X_INTEGRAL           0x18        //int16_t  accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
//上次读取IIC数据后，Y轴速度积分后所得值
#define PIXEL_FLOW_Y_INTEGRAL           0x1a        //int16_t  accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
//上次读取IIC数据后X轴角速度积分值
#define GYRO_X_RATE_INTEGRAL            0x1c        //int16_t  accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]  
//上次读取IIC数据后Y轴角速度积分值
#define GYRO_Y_RATE_INTEGRAL            0x1e        //int16_t  accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
//上次读取IIC数据后Z轴角速度积分值
#define GYRO_Z_RATE_INTEGRAL            0x20        //int16_t  accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
//上次和这次读取IIC数据的时间间隔
#define INTEGRATION_TIMESPAN            0x22        //uint32_t accumulation timespan in microseconds since last I2C readout [microseconds]
//超声波数据上次更新到现在的时间间隔
#define SONAR_TIMESTAMP2                 0x26        //uint32_t time since last sonar update [microseconds]
//地面距离
#define GROUND_DISTANCE2                0x2a        //int16_t  Ground distance in meters*1000 [meters*1000]
//陀螺仪温度
#define GYRO_TEMPERATURE                0x2c        //int16_t  Temperature * 100 in centi-degrees Celsius [degcelsius*100] 
//光流积分数据质量
#define QUALITY                         0x2e        //uint8_t averaged quality of accumulated flow values [0:bad quality;255: max quality]


//读指定寄存器指定字节数数据
u8 flow_read_data(u8 addr,u8 reg,u8 len,u8 *buf);
//读8位无符号数据
uint8_t     readu8_date(u8 addr,u8 reg);
//读16位无符号数据
uint16_t    readu16_date(u8 addr,u8 reg);
//读16位有符号数据
int16_t     reads16_date(u8 addr,u8 reg);
//读32位无符号数据
uint32_t    readu32_date(u8 addr,u8 reg);
//更新光流数据
void FLOW_getData();

extern unsigned char flowdata[47];
extern short int X_Speed,Y_Speed;
extern float Xmove,Ymove;

#endif
