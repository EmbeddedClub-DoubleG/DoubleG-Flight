使用了LED的地方：
（注：注释在这里表示在有三个led的电路板上led在这里有被使用，但是在只有redled的板上，我把有些使用led的代码注释了）
----------------------------------
LEDRed_ON()：（注：这句话表示下面这些是红灯的使用的地方，之所以放一个函数在这里，是为了在用这个函数的时候好找参考）
1.刚启动时，在进入大循环之前，led常亮
2.查询AT45DB 是否在电路板上，如果不在，led闪烁：LEDRed_ON();delay_ms(100);LEDRed_OFF();delay_ms(100);
//3.处理pc的命令的时候：LED_Set_Blink(Red,100,100,4);  // 红色的LED 闪烁表示正在处理 PC发送的命令
//4.进入电调校准模式的时候：LED_Set_Blink(Red,150,150,3);  //提示正在设置电调行程
5.任务模式的前3s的静止时间内led常亮：LEDRed_ON();//在这静止的3秒中，红灯常亮
----------------------------------
LEDBlue_ON()：
//1.数据上传上位机的时候：LED_Set_Blink(Blue,60,100,1);
//2.切换飞行模式的时候，如果原先的飞行模式是手动模式或平衡模式或定点模式，则亮灯：//LED_Set_Blink(Blue,100,200,4);
3.（这里已经被我改成红led）led指示当前飞行模式：
	case Quad_ESC_Cal: LED_Set_Blink(Red,50,50,10); 	
		break;	//电调校准模式，红色的LED快速闪烁
	case Quad_Manual: LED_Set_Blink(Red,100,100,1);
		break;	//手动模式  红色的LED 1S 闪一次
	case Quad_Level_Lock: LED_Set_Blink(Red,100,100,2);
		break;	//平衡模式  红色的LED 1S 闪两次
	case Quad_Hold_Position: LED_Set_Blink(Red,100,100,3);
		break;  //定点模式  红色的LED 1S 闪三次
	case Quad_Assignment: LED_Set_Blink(Red,100,100,4);
		break;  //任务模式  红色的LED 1S 闪四次		
	case Quad_Landing: LED_Set_Blink(Red,100,100,5);
		break;  //降落模式  红色的LED 1S 闪5次		
//4.标记遥控器输入中立值：//LED_Set_Blink(Blue,50,50,4);
//5.保持上位机发送的PID参数：//LED_Set_Blink(Blue,50,50,4); //LED 闪烁
----------------------------------
LEDGreen_ON()：
1.（这里已经被我改成红led）四轴锁定的时候：LED_Set_Blink(Red,1000,1000,1); //绿色LED长亮 表示PWM锁定中




