/* AT45DB.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-25
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用资源：
1. SPI2 接口 
	PB12  SS
	PB13  SCK
	PB14  MISO
	PB15  MOSI

功能：
提供AT45DB  FLASH 存储接口操作API 。
------------------------------------
 */

#include "AT45DB.h"
#include "common.h"

volatile struct data_map Config;


/**************************实现函数********************************************
*函数原型:		void AT45db161_SPI_Configuration(void)
*功　　能:	    初始化 AT45DB 的SPI 接口
*******************************************************************************/
void AT45db161_SPI_Configuration(void){

	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* SCK, MISO and MOSI  PB13=CLK,PB14=MISO,PB15=MOSI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //开启上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
	/*  PB.12 作片选*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);//预置为高

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //TF卡片选。PC6
	GPIO_SetBits(GPIOC, GPIO_Pin_6);//预置为高

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //TF卡插入检测 PC13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //开启上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	PWR_BackupAccessCmd(ENABLE);//允许修改RTC 和后备寄存器
	RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。
	//BKP_TamperPinCmd(DISABLE);//关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用
	PWR_BackupAccessCmd(DISABLE);//禁止修改后备寄存器

	/* SPI2 configuration  */
	SPI_Cmd(SPI2, DISABLE);        
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //主
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_Low;       //CPOL=0 时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;       //CPHA=0 数据捕获第1个
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //软件NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //2分频

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
    
	SPI_Init(SPI2, &SPI_InitStructure);	 //应用配置到 SPI2
	SPI_Cmd(SPI2, ENABLE); 
}

void SPI2_SetSpeed(uint16_t SpeedSet){
	SPI2->CR1 &= 0XFF87; 
	SPI2->CR1 |= SpeedSet;	//设置SPI2速度  
	SPI2->CR1 |= 1<<6; 		//SPI设备使能
}

/************************************************************************
** 函数名称:static u8 SPI_ReadWrite_Byte(u8 byte)
** 功能描述:  发送或者接收1个字节
** 输　入:    byte    发送时候,byte传递为发送的数据字节， 接收的时候，则固定为0xff
** 输　出:    SPI1->DR  发送时候，可以忽略, 接收的时候，则为接收数据
***********************************************************************/
uint8_t SPI2_ReadWrite_Byte(uint8_t byte)
{
	/*等待发送寄存器空*/
	while((SPI2->SR & SPI_I2S_FLAG_TXE)==RESET);
	/*发送一个字节*/
	SPI2->DR = byte;
	/* 等待接收寄存器有效*/
	while((SPI2->SR & SPI_I2S_FLAG_RXNE)==RESET);
	return(SPI2->DR);
}

/**************************实现函数********************************************
*函数原型:		uint8_t AT45DB_IS_BUSY(void)
*功　　能:	    查询AT45DB 是否准备好接收数据
	返回 1   表示 AT45DB 已准备好 可以进行下一步操作
	返回 0   表示 AT45DB 正忙
*******************************************************************************/
uint8_t AT45DB_IS_BUSY(void)
{
	uint8_t Status_Register;

	AT45db161_SPI_SELECT(); //片选Dataflash
	//delay_us(1);
	SPI2_ReadWrite_Byte(AT45DB_READ_STATE_REGISTER);
	Status_Register = SPI2_ReadWrite_Byte(0x00);
	AT45db161_SPI_DESELECT();
	if(Status_Register & 0x80)
		return 1; //If bit seven is a one, then the device is ready 
	return 0;
}

/**************************实现函数********************************************
*函数原型:		uint8_t AT45DB_Check(void)
*功　　能:	    查询AT45DB 是否在电路板上
读取 AT45DB 的ID  检查三个字节是否正确
	返回 1   表示 AT45DB 在板上
	返回 0   表示 AT45DB ID错误
*******************************************************************************/
uint8_t AT45DB_Check(void) {
	u8 buf[3];
	AT45db161_SPI_SELECT(); //片选Dataflash
	SPI2_ReadWrite_Byte(0x9F);
	buf[0] = SPI2_ReadWrite_Byte(0);
	buf[1] = SPI2_ReadWrite_Byte(0);
	buf[2] = SPI2_ReadWrite_Byte(0);
	AT45db161_SPI_DESELECT();
	if((buf[0] == 0x1F)&&(buf[1] == 0x26)&&(buf[2] == 0x0))
		return 1; //ID 正确
	return 0;
}

/**************************实现函数********************************************
*函数原型:		void AT45DB_ReadPage(uint16_t Page_Add,uint8_t *pdata)
*功　　能:	    读一页数据  从 AT45DB 的 Page_Add 开始
输入   uint16_t Page_Add   页地址   0-4096
	   uint8_t *pdata   数组首地址	
*******************************************************************************/
void AT45DB_ReadPage(uint16_t Page_Add,uint8_t *pdata)
{
	int i;
	if(Page_Add > 4096) return;
	while(AT45DB_IS_BUSY()==0);
	
	AT45db161_SPI_SELECT(); 
	SPI2_ReadWrite_Byte(AT45DB_MM_PAGE_READ); //直接读存储器
	SPI2_ReadWrite_Byte((int8_t)(Page_Add >> 6)); //A20 - A9
	SPI2_ReadWrite_Byte((int8_t)(Page_Add << 2));
	SPI2_ReadWrite_Byte(0x00); //three address bytes

	SPI2_ReadWrite_Byte(0x00); // four don’t care bytes
	SPI2_ReadWrite_Byte(0x00);
	SPI2_ReadWrite_Byte(0x00);
	SPI2_ReadWrite_Byte(0x00);
	for (i=0; i<512; i++)
	{
		*pdata++ = SPI2_ReadWrite_Byte(0x00);
	}
	AT45db161_SPI_DESELECT(); 
}

/**************************实现函数********************************************
*函数原型:		void AT45DB_WritePage(uint16_t page,uint8_t *Data)
*功　　能:	    写一页数据到 AT45DB
输入   uint16_t page   页地址   0-4096
	   uint8_t *Data   数组首地址	
*******************************************************************************/
void AT45DB_WritePage(uint16_t page,uint8_t *Data)  //写一整页，页范围0-4095
{
	uint16_t i;
	while(AT45DB_IS_BUSY()==0);

	AT45db161_SPI_SELECT(); //片选Dataflash if (buffer_num == FLASH_BUFFER1)
	//delay_us(1);

	SPI2_ReadWrite_Byte(AT45DB_BUFFER_2_WRITE);
	SPI2_ReadWrite_Byte(0x00);
	SPI2_ReadWrite_Byte(0x00);
	SPI2_ReadWrite_Byte(0x00);
	for (i = 0;i < 512; i++)
	{
		SPI2_ReadWrite_Byte(Data[i]);
	}
	AT45db161_SPI_DESELECT(); 

	while(AT45DB_IS_BUSY()==0);  
	if ( page < 4096)
	{
		AT45db161_SPI_SELECT(); //片选Dataflash if (buffer_num == FLASH_BUFFER1)
		SPI2_ReadWrite_Byte(AT45DB_B2_TO_MM_PAGE_PROG_WITH_ERASE);
		SPI2_ReadWrite_Byte((u8)(page>>6));
		SPI2_ReadWrite_Byte((u8)(page<<2));
		SPI2_ReadWrite_Byte(0x00);
		AT45db161_SPI_DESELECT();  
	}
}

/**************************实现函数********************************************
*函数原型:		void AT45DB_Read_Bytes(uint32_t add,uint8_t *pdata, uint16_t len)
*功　　能:	    从add 位置开始 读取多个字节
输入   uint32_t add     读取地址   0-4096*512
	   uint8_t *pdata   存放读取数据的数组首地址	
	   uint16_t len     要读取的字节数
*******************************************************************************/
void AT45DB_Read_Bytes(uint32_t add,uint8_t *pdata, uint16_t len)
{
	int i,j,k;
	uint8_t temp[512];
	uint16_t page,offset;
	if(add > 4096*512) return;
	page = add / 512;
	offset = add % 512;

	AT45DB_ReadPage(page++,temp);
	for(i = offset ; ( len!=0 )&&(i<512);len--,i++ )
	{
		 *pdata++ = temp[i];
	}
	
	if((i == 512)&&(len != 0)){  //超过一个页的范围了

		if(len > 512){

			for(k=0;k<len/512;k++) //读多页
			{
			AT45DB_ReadPage(page++,temp);
			for(j=0;j<512;j++)
			{
				*pdata++ = temp[j];
			}
			}
		    len -= 512*k;
		   }
		if(len != 0)  //最后一小部分
		{
			AT45DB_ReadPage(page,temp);
			for (j = 0;j<len ; j++)
			{
				*pdata++ = temp[j];
			}
		}
	}
}

/**************************实现函数********************************************
*函数原型:		void AT45DB_Write_Bytes(uint32_t add,uint8_t *pdata, uint16_t len)
*功　　能:	    从add 位置开始 写入多个字节
输入   uint32_t add     写入地址   0-4096*512
	   uint8_t *pdata   存放要写入数据的数组首地址	
	   uint16_t len     要写入的字节数
*******************************************************************************/	
void AT45DB_Write_Bytes(uint32_t add,uint8_t *pdata, uint16_t len){
	uint8_t temp[512];
	uint16_t page,offset;
	uint16_t  i,j,k;
	if(add > 4096*512) return;
	page = add / 512;
	offset = add % 512;
	
	AT45DB_ReadPage(page,temp);
	for(i = offset ; ( len!=0 )&&(i<512);len--,i++ )
	{
		temp[i] = *pdata++;
	}
	AT45DB_WritePage(page++,temp);
  
	if((i == 512)&&(len != 0)){  //超过一个页的范围了
		
		if(len > 512){
			for(k=0;k<len/512;k++) //写多页
			{
			for(j=0;j<512;j++)
			{
				temp[j] = *pdata++;
			}
			AT45DB_WritePage(page++,temp);
		}
		len -= 512*k;
		}
		
		if(len != 0) //仍然有一小部分还没有写完。 
		{
			AT45DB_ReadPage(page,temp);
			for (j = 0;j<len ; j++)
			{
				temp[j] = *pdata++;
			}
			AT45DB_WritePage(page,temp);
		}
		
	}
}	

/**************************实现函数********************************************
*函数原型:		void AT45DB_Write_float(uint32_t add, float wvalue)
*功　　能:	    将一个浮点数写入add 位置 占用4个字节的空间
输入   uint32_t add     写入地址   0-4096*512
	   float wvalue     要写入的值
*******************************************************************************/
void AT45DB_Write_float(uint32_t add, float wvalue)
{
	f_bytes data;
	data.value = wvalue;
	AT45DB_Write_Bytes(add,&data.byte[0],4);
}

/**************************实现函数********************************************
*函数原型:		float AT45DB_Read_float(uint32_t add)
*功　　能:	    从add 地址读出一个浮点数
输入   uint32_t add     读出始地址   0-4096*512
返回 读出浮点值
*******************************************************************************/
float AT45DB_Read_float(uint32_t add)
{
	f_bytes data;
	data.value = 0;
	AT45DB_Read_Bytes(add,&data.byte[0],4);
	return data.value;
}

/**************************实现函数********************************************
*函数原型:		void AT45DB_Write_int16(uint32_t add, int16_t wvalue)
*功　　能:	    将一个整数写入add 位置 占用2个字节的空间
输入   uint32_t add     写入地址   0-4096*512
	   int16_t wvalue   要写入的值
*******************************************************************************/
void AT45DB_Write_int16(uint32_t add, int16_t wvalue)
{
  i_bytes data;
  data.value = wvalue;
  AT45DB_Write_Bytes(add,&data.byte[0],2);
}

/**************************实现函数********************************************
*函数原型:		int16_t AT45DB_Read_int16(uint32_t add)
*功　　能:	    从add 地址读出一个整数
输入   uint32_t add     读出地址   0-4096*512
返回 读出整型值
*******************************************************************************/
int16_t AT45DB_Read_int16(uint32_t add)
{
	i_bytes data;
	data.value = 0;
	AT45DB_Read_Bytes(add,&data.byte[0],2);
	return data.value;
}

/**************************实现函数********************************************
*函数原型:		void Load_Default_config(void)
*功　　能:	    装载默认的参数，默认的PID参数和偏置
*******************************************************************************/
void Load_Default_config(void){
	Config.Available = (int16_t)0xA55A;	//数据有效标志

	Config.Angle_Roll_P = 4.5f;	//以下PID参数适合450轴距的四轴机架。
	Config.Angle_Roll_I = 0.0f;
	Config.Angle_Roll_D = 0.0f;

	Config.Angle_Pitch_P = 4.5f;
	Config.Angle_Pitch_I = 0.0f;
	Config.Angle_Pitch_D = 0.0f;

	Config.Angle_Yaw_P = 4.5f;
	Config.Angle_Yaw_I = 0.0f;
	Config.Angle_Yaw_D = 0.0f;

	Config.Rate_Roll_P = 0.15f;
	Config.Rate_Roll_I = 0.1f;
	Config.Rate_Roll_D = 0.07f;

	Config.Rate_Pitch_P = 0.15f;
	Config.Rate_Pitch_I = 0.1f;
	Config.Rate_Pitch_D = 0.07f;

	Config.Rate_Yaw_P = 0.2f;
	Config.Rate_Yaw_I = 0.02f;
	Config.Rate_Yaw_D = 0.0f;

	Config.High_P = 1.5f;
	Config.High_I = 0.001f;
	Config.High_D = 0.0f;

	Config.Climb_High_P = 0.75f;
	Config.Climb_High_I = 0.0f;
	Config.Climb_High_D = 1.0f;

	Config.Position_Hold_P = 2.8f;
	Config.Position_Hold_I = 0.12f;
	Config.Position_Hold_D = 0.069f;

	Config.Position_Speed_P = 0.0f;
	Config.Position_Speed_I = 0.0f;
	Config.Position_Speed_D = 0.0f;

	Config.target_hight	= 0.8f;

	Config._Roll_offset = 0.0f;
	Config._Pitch_offset = 0.0f;

	Config.pwm_in_offset1 = 1500; //初始中立值 1500
	Config.pwm_in_offset2 = 1500;
	Config.pwm_in_offset3 = 1000;
	Config.pwm_in_offset4 = 1500;

	Config.Magnetic_offset_X = 0;
	Config.Magnetic_offset_Y = 0;
	Config.Magnetic_offset_Z = 0;

	Config.Magnetic_Scale_X = 1.0f;
	Config.Magnetic_Scale_Y = 1.0f;
	Config.Magnetic_Scale_Z = 1.0f;
	Config.ACC_z_zero = 9.8f;  //重力加速度，默认为9.8， 需要做标定。/
	Config.fly_mode = QUADP;  //默认十字模式的四轴，此参数可以在上位机修改
	Config.yaw_dir = 1;		  //航向反向
	Config.GPS_baud = 0;
}

/**************************实现函数********************************************
*函数原型:		void AT45DB_Write_config(void)
*功　　能:	    将当前的设置保存到Flash，永久保存
*******************************************************************************/
void AT45DB_Write_config(void){
	Config.Available = (int16_t)0xA55A;	 //数据有效标志

	Config.Angle_Roll_P = Stabilize_Roll.Kp;
	Config.Angle_Roll_I = Stabilize_Roll.Ki;
	Config.Angle_Roll_D = Stabilize_Roll.Kd;

	Config.Angle_Pitch_P = Stabilize_Pitch.Kp;
	Config.Angle_Pitch_I = Stabilize_Pitch.Ki;
	Config.Angle_Pitch_D = Stabilize_Pitch.Kd;

	Config.Angle_Yaw_P = Stabilize_Yaw.Kp;
	Config.Angle_Yaw_I = Stabilize_Yaw.Ki;
	Config.Angle_Yaw_D = Stabilize_Yaw.Kd;

	Config.Rate_Roll_P = RollRate.Kp;
	Config.Rate_Roll_I = RollRate.Ki;
	Config.Rate_Roll_D = RollRate.Kd;

	Config.Rate_Pitch_P = PitchRate.Kp;
	Config.Rate_Pitch_I = PitchRate.Ki;
	Config.Rate_Pitch_D = PitchRate.Kd;

	Config.Rate_Yaw_P = YawRate.Kp;
	Config.Rate_Yaw_I = YawRate.Ki;
	Config.Rate_Yaw_D = YawRate.Kd;

	Config.High_P = AutoHigh_THR.Kp;
	Config.High_I = AutoHigh_THR.Ki;
	Config.High_D = AutoHigh_THR.Kd;

	Config.Climb_High_P = Climb.Kp;
	Config.Climb_High_I = Climb.Ki;
	Config.Climb_High_D = Climb.Kd;

	Config.Position_Hold_P = Position_Hold.Kp;
	Config.Position_Hold_I = Position_Hold.Ki;
	Config.Position_Hold_D = Position_Hold.Kd;

	Config.Position_Speed_P = Position_Speed.Kp;
	Config.Position_Speed_I = Position_Speed.Ki;
	Config.Position_Speed_D = Position_Speed.Kd;
	
	Config.pwm_in_offset1 = PWM_Offset_Roll; // Roll输入中立位
	Config.pwm_in_offset2 = PWM_Offset_Pitch;// Pitch输入中立位
	Config.pwm_in_offset4 = PWM_Offset_Yaw;	 // Yaw输入中立位

	//写入Flash 
	AT45DB_Write_Bytes(Config_add,(uint8_t*)&Config,sizeof(Config));
	Initial_Quadrotor_Math(); // 重新提取PID数据
}
																	
/**************************实现函数********************************************
*函数原型:		void AT45DB_Read_config(void)
*功　　能:	    读取配置，当配置表无效的时候，装载默认值。
*******************************************************************************/
void AT45DB_Read_config(void){
	static u8 isreaded = 0;	 //上电只读取一次配置。
	if(isreaded)return;
	isreaded = 1;
	AT45DB_Read_Bytes(Config_add,(uint8_t*)&Config,sizeof(Config));
	if(Config.Available != (int16_t)0xA55A){//配置表是否有效？
		Load_Default_config();	//装载默认的配置，并将默认值写入Flash
		AT45DB_Write_Bytes(Config_add,(uint8_t*)&Config,sizeof(Config));
		}
}
	
//------------------End of File----------------------------		
