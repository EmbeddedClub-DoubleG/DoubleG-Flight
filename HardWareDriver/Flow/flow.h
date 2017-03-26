#ifndef __FLOW_H
#define __FLOW_H
#include "common.h"
//������ַ
#define FLOW_ADDR  0x42
//�洢���궨�壬������2�ֽ�1�����ݣ����ֽڵص�λ�����ֽڸ�λ

//δ���ֵ�����
//��¼�ܵĴ�����IIC��֡��
#define FRAME_COUNT_SUM     0x00                    //uint16_t counts created I2C frames [frames]
//X������һ֡���������ƶ���*10
#define PIXEL_FLOW_X_SUM    0x02                    //int16_t latest x flow measurement in pixels*10 [pixels]
//Y������һ֡���������ƶ���*10
#define PIXEL_FLOW_Y_SUM    0x04                    //int16_t latest y flow measurement in pixels*10 [pixels]
//X���ٶ�
#define FLOW_COMP_M_X       0x06                    //int16_t x velocity*1000 [meters/sec]
//Y���ٶ�
#define FLOW_COMP_M_Y       0x08                    //int16_t y velocity*1000 [meters/sec]
//����ͼ������
#define QUAL                0x0a                    //int16_t Optical flow quality / confidence [0: bad, 255: maximum quality]
//X���������ٶ�
#define GYRO_X_RATE         0x0c                    //int16_t latest gyro x rate [rad/sec]
//Y���������ٶ�
#define GYRO_Y_RATE         0x0e                    //int16_t latest gyro y rate [rad/sec]
//Z���������ٶ�
#define GYRO_Z_RATE         0x10                    //int16_t latest gyro z rate [rad/sec]
//���������ݷ�Χ
#define GYRO_RANGE          0x12                    //uint8_t gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
//�����������ϴθ��µ����ڵ�ʱ������
#define SONAR_TIMESTAMP1     0x13                    //uint8_t time since last sonar update [milliseconds] 
//��������  ��ֵ����֪����   ��ֵ��δ֪����
#define GROUND_DISTANCE1    0x14                    //int16_t Ground distance in meters*1000 [meters]. Positive value: distance known. 
                                                            //Negative value: Unknown distance
//���ֺ�������
//�ϴζ�ȡ���ݺ������ݸ����˶��ٴ�
#define FRAME_COUNT_SINCE_LAST_READOUT  0x16        //uint16_t number of flow measurements since last I2C readout [frames]
//�ϴζ�ȡIIC���ݺ���X���ٶȻ��ֺ�����ֵ
#define PIXEL_FLOW_X_INTEGRAL           0x18        //int16_t  accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
//�ϴζ�ȡIIC���ݺ���Y���ٶȻ��ֺ�����ֵ
#define PIXEL_FLOW_Y_INTEGRAL           0x1a        //int16_t  accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
//�ϴζ�ȡIIC���ݺ�X�����ٶȻ���ֵ
#define GYRO_X_RATE_INTEGRAL            0x1c        //int16_t  accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]  
//�ϴζ�ȡIIC���ݺ�Y�����ٶȻ���ֵ
#define GYRO_Y_RATE_INTEGRAL            0x1e        //int16_t  accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
//�ϴζ�ȡIIC���ݺ�Z�����ٶȻ���ֵ
#define GYRO_Z_RATE_INTEGRAL            0x20        //int16_t  accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
//�ϴκ����ζ�ȡIIC���ݵ�ʱ������
#define INTEGRATION_TIMESPAN            0x22        //uint32_t accumulation timespan in microseconds since last I2C readout [microseconds]
//�����������ϴθ��µ����ڵ�ʱ������
#define SONAR_TIMESTAMP2                 0x26        //uint32_t time since last sonar update [microseconds]
//��������
#define GROUND_DISTANCE2                0x2a        //int16_t  Ground distance in meters*1000 [meters*1000]
//�������¶�
#define GYRO_TEMPERATURE                0x2c        //int16_t  Temperature * 100 in centi-degrees Celsius [degcelsius*100] 
//����������������
#define QUALITY                         0x2e        //uint8_t averaged quality of accumulated flow values [0:bad quality;255: max quality]


//��ָ���Ĵ���ָ���ֽ�������
u8 flow_read_data(u8 addr,u8 reg,u8 len,u8 *buf);
//��8λ�޷�������
uint8_t     readu8_date(u8 addr,u8 reg);
//��16λ�޷�������
uint16_t    readu16_date(u8 addr,u8 reg);
//��16λ�з�������
int16_t     reads16_date(u8 addr,u8 reg);
//��32λ�޷�������
uint32_t    readu32_date(u8 addr,u8 reg);
//���¹�������
void FLOW_getData(void);

extern unsigned char flowdata[47];
extern short int X_Speed,Y_Speed;
extern float Xmove,Ymove;

#endif
