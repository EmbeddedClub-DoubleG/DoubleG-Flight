
#include "MPU6500.h"


#define Buf_SIZE  10	   //保存最近 10组数据


int16_t  MPU6500_FIFO[7][Buf_SIZE];
uint8_t Gyro_Off_started = 0;
int16_t lastAx,lastAy,lastAz,lastGx,lastGy,lastGz;
static int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
static uint8_t	 Wr_Index = 0;


//添加一个新的值到 队列 进行滤波
static void MPU6500_NewVal(int16_t* buf,int16_t val) {
  	buf[Wr_Index] = val;
}

static int16_t MPU6500_GetAvg(int16_t* buf){
    int i;
	int32_t	sum = 0;
	for(i=0;i<Buf_SIZE;i++)
		sum += buf[i];
	sum = sum / Buf_SIZE;
	return (int16_t)sum;
}

static u8 MPU6500_readReg(u8 reg){
	u8 data;
	MPU6500_CSL();
	SPI3_ReadWrite_Byte(reg|0x80);
	data = SPI3_ReadWrite_Byte(0xff);
	MPU6500_CSH();
	return data;	
}

static void MPU6500_writeReg(u8 reg, u8 data){
	MPU6500_CSL();
	SPI3_ReadWrite_Byte(reg);
	SPI3_ReadWrite_Byte(data);
	MPU6500_CSH();	
}

u8 MPU6500_readID(void){
	return MPU6500_readReg(MPU6500_RA_WHO_AM_I);
}

void MPU6500_initialize(void){ //初始化
	MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1,0x80); //Reset ing
	delay_ms(110);
	MPU6500_writeReg(MPU6500_RA_SIGNAL_PATH_RESET,0x07);
	delay_ms(110);					 
	MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1,0x00); //Internal 20MHz oscillator 
	delay_ms(1);
	MPU6500_writeReg(MPU6500_RA_ACCEL_CONFIG,0x00);//加速度度最大量程 +-2G
	delay_ms(1);
	MPU6500_writeReg(0x1D,0x02);  //带宽91
	delay_ms(1);
	MPU6500_writeReg(MPU6500_RA_GYRO_CONFIG,0x18); //陀螺仪最大量程 +-2000度每秒
	delay_ms(1);
	MPU6500_writeReg(MPU6500_RA_PWR_MGMT_2,0x00); 
	delay_ms(1);
	Gx_offset=AT45DB_Read_int16(AT_Gyro_offsetx);
	Gy_offset=AT45DB_Read_int16(AT_Gyro_offsety);
	Gz_offset=AT45DB_Read_int16(AT_Gyro_offsetz);
}

//读取陀螺仪数据
void MPU6500_readGyro_Acc(int16_t *gyro,int16_t *acc){
	static u8 buf[14];
	static int16_t gx,gy,gz;

	MPU6500_CSL();
	SPI3_readRegs(MPU6500_RA_ACCEL_XOUT_H,14,buf);
	MPU6500_CSH();

	//acc
	MPU6500_NewVal(&MPU6500_FIFO[0][0],(int16_t)(((int16_t)buf[0]) << 8 | buf[1]));
	MPU6500_NewVal(&MPU6500_FIFO[1][0],(int16_t)(((int16_t)buf[2]) << 8 | buf[3]));
	MPU6500_NewVal(&MPU6500_FIFO[2][0],(int16_t)(((int16_t)buf[4]) << 8 | buf[5]));
	//temp
	MPU6500_NewVal(&MPU6500_FIFO[3][0],(int16_t)(((int16_t)buf[6]) << 8 | buf[7]));
	//gyro
	MPU6500_NewVal(&MPU6500_FIFO[4][0],(int16_t)(((int16_t)buf[8]) << 8 | buf[9]));
	MPU6500_NewVal(&MPU6500_FIFO[5][0],(int16_t)(((int16_t)buf[10]) << 8 | buf[11]));
	MPU6500_NewVal(&MPU6500_FIFO[6][0],(int16_t)(((int16_t)buf[12]) << 8 | buf[13]));
	
	Wr_Index = (Wr_Index + 1) % Buf_SIZE;


	gx =  MPU6500_GetAvg(&MPU6500_FIFO[4][0]);
	gy =  MPU6500_GetAvg(&MPU6500_FIFO[5][0]);
	gz =  MPU6500_GetAvg(&MPU6500_FIFO[6][0]);
	gyro[0] = gx - Gx_offset;	//gyro
	gyro[1] = gy - Gy_offset;
	gyro[2] = gz - Gz_offset;
			  
	acc[0] = MPU6500_GetAvg(&MPU6500_FIFO[0][0]); //acc
	acc[1] = MPU6500_GetAvg(&MPU6500_FIFO[1][0]);
	acc[2] = MPU6500_GetAvg(&MPU6500_FIFO[2][0]);
	
	
	  
	lastGx = gyro[0];
	lastGy = gyro[1];
	lastGz = gyro[2];
	lastAx = acc[0];
	lastAy = acc[1];
	lastAz = acc[2];
	
}


/**************************实现函数********************************************
*函数原型:		void MPU6500_InitGyro_Offset(void)
*功　　能:	    读取 MPU6500的陀螺仪偏置
此时模块应该被静止放置。以测试静止时的陀螺仪输出
*******************************************************************************/
void MPU6500_InitGyro_Offset(void)
{
	unsigned int i;
	int16_t temp[3],temp2[3];
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	Gx_offset=0;
	Gy_offset=0;
	Gz_offset=0;
	Gyro_Off_started = 1;

	for(i=0;i<50;i++){
  		delay_us(100);
		MPU6500_readGyro_Acc(temp,temp2);
	}
 	for(i=0;i<1000;i++){
		delay_us(200);
		MPU6500_readGyro_Acc(temp,temp2);
		tempgx+= temp[0];
		tempgy+= temp[1];
		tempgz+= temp[2];
	}
	Gx_offset=tempgx/1000;//MPU6500_FIFO[3][10];
	Gy_offset=tempgy/1000;//MPU6500_FIFO[4][10];
	Gz_offset=tempgz/1000;//MPU6500_FIFO[5][10];

	AT45DB_Write_int16(AT_Gyro_offsetx,Gx_offset);
	AT45DB_Write_int16(AT_Gyro_offsety,Gy_offset);
	AT45DB_Write_int16(AT_Gyro_offsetz,Gz_offset);
	Gyro_Off_started = 0;

}


//------------------End of File----------------------------

