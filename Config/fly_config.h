#ifndef __FLY_CONFIG_H
#define __FLY_CONFIG_H


//飞行系统用到的中断程序优先级设置
#define NVIC_GROUPS	  		  NVIC_PriorityGroup_1

// 
#define NVIC_USART1_PRI        7
#define NVIC_PWM_Input_CH1_PRI 2 
#define NVIC_PWM_Input_CH2_PRI 3
#define NVIC_PWM_Input_CH3_PRI 4
#define NVIC_PWM_Input_CH4_PRI 5
#define NVIC_PWM_Input_CH5_PRI 6
#define NVIC_PWM_Input_CH6_PRI 8
#define NVIC_UART3_GPS_PRI     13
#define NVIC_Ultrasonic_PRI   10

#define GPS_Baudrate  115200L

//飞行器类型   请在上位机更改飞行器类型
#define QUADP	 0x01  //十字型四轴
#define QUADX	 0x02  //X字型四轴
#define Y4		 0x03
#define Y6		 0x04
#define HEX6     0x05
#define HEX6X	 0x06
#define OCTOX8	 0x07

#define default_quad  QUADP  //飞行器的架构，默认十字四轴 [建议使用上位机配置]

//最低油门
#define MINTHROTTLE 1020
//最高油门
#define MAXTHROTTLE 1850

#define YAW_DIRECTION -1 // 当航向需要反向的时候，设置这个值。[建议使用上位机配置]
//#define YAW_DIRECTION 1
#define Targe_high   0.8f    //目标高度。在定点飞行模式时会用到

//是否使用遥控器来做参数配置  1 使用， 0 为不使用/
//注意，当使用这个功能后，起飞前需要解锁油门才能进行飞行。解锁方式见 main.c中
// 在此感谢[李老板没钱了]增加这个实用的功能。
#define Remote_Config_Enable    1

//xiang:这是我自己加的。这一段判断使用的是什么上位机来进行条件编译
#define Yingzhang_GCS 1
#define Captain_GCS 0
#define SerialDebug 0//update20161227:增加串口调试条件编译

#endif
//------------------End of File----------------------------
