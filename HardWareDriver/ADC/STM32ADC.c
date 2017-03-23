/* STM32ADC.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-11-12
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用STM32 资源：
1. 使用ADC1进行模数转换
2. 使用DMA 通道1采集ADC的结果，不需要程序的干预

------------------------------------
 */

#include "STM32ADC.h"

#define ADC1_DR_Address    ((uint32_t)0x4001204C)

uint16_t AD_Value[2];

//ADC 引脚配置
void ADC_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	//启动GPIOB	时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB ,
                           ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    //ADC_CH8--> PB0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //配置引脚为 模拟输入      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//ADC_CH9--> PB1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

/*******************************************************************************
* Function Name  : ADCDMA_Configuration
* Description    : DMA设置：从ADC模块自动读转换结果至内存
*******************************************************************************/
void ADC_DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    //启动DMA时钟	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    //DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&AD_Value;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    //BufferSize=2，因为ADC转换序列有2个通道
    //如此设置，使序列1结果放在AD_Value[0]，序列2结果放在AD_Value[1]
    DMA_InitStructure.DMA_BufferSize = 2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //循环模式开启，Buffer写满后，自动回到初始地址开始传输
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    //配置完成后，启动DMA通道
    DMA_Cmd(DMA2_Stream0, ENABLE);
}

/*******************************************************************************
* Function Name  : ADC1_Configuration
* Description    : ADC1设置（包括ADC模块配置和自校准）
*******************************************************************************/
void ADC1_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	//启动ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | 
                         RCC_APB2Periph_ADC3, ENABLE);
	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;  //ADC独立模式
  	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;  
  	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; //ADC采样周期2分频
  	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//连续转换开启
  	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfConversion = 2;	//设置转换序列长度为2
  	ADC_Init(ADC1, &ADC_InitStructure);
    
    //常规转换序列1：通道8  AIN8
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_480Cycles);
    //常规转换序列2：通道9，采样时间>2.2us,(239cycles)	AIN9
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_480Cycles);
    
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	//ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    // 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
    ADC_DMACmd(ADC1, ENABLE);
    
}

/*******************************************************************************
* Function Name  : ADC_Voltage_initial
* Description    : ADC 初始化程序，上电的时候调用一次。
*******************************************************************************/
void ADC_Voltage_initial(void){
	ADC_GPIO_Configuration();
	ADC_DMA_Configuration();
	ADC1_Configuration();
	
	//启动第一次AD转换
    /* Start ADC1 Software Conversion */ 
  	ADC_SoftwareStartConv(ADC1);
    //因为已经配置好了DMA，接下来AD自动连续转换，结果自动保存在AD_Value处 
}

/*******************************************************************************
* Function Name  : Get_Bat_Vol
* Description    : 读取电池电压，单位 0.1V   
*******************************************************************************/
int16_t Get_Bat_Vol(void){
	float temp;
	temp = AD_Value[1];
	temp = (temp*330)/4096;	//ADC值 到电压的转换
	temp = temp * 11; //分压电阻 比例
	return temp /10;  
}

/*******************************************************************************
* Function Name  : Get_Sevor_Vol
* Description    : 读取舵机电压，单位 0.1V   
*******************************************************************************/
int16_t Get_Sevor_Vol(void){
	float temp;
	temp = AD_Value[0];
	temp = (temp*330)/4096;	//ADC值 到电压的转换
	temp = temp * 11; //分压电阻 比例
	return temp /10;
}

//------------------End of File----------------------------
