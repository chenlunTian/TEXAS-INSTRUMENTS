/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: systems.c
**
** 创建人员: CLTian
**
** 创建日期: 2019-07-23
**
** 文档描述: 
**
**----------------------------------版本信息------------------------------------
** 版本代号: V0.1
**
** 版本说明: 初始版本
**
**------------------------------------------------------------------------------
*******************************************************************************/
#include "Headfile.h"
#include "systems.h"


void SystemClockInit(void)
{
		SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
		initTime();	//系统滴答	
}

void HardWave_Init(void)
{
	SystemClockInit();
	ConfigureUART0();//串口0初始化，山外地面站
	ADC_Init();//ADC初始化，检测外部电池电压
  LED_Init();//LED状态指示灯初始化
  Key_Init();//板载按键初始化
  Init_I2C();//硬件I2C1初始化
//  ConfigureUART1();//串口1初始化
  ConfigureUART3();//串口3初始化
  ConfigureUART7();//串口7初始化
  ConfigureUART6();//串口6初始化
  Time_init();//调度定时器初始化
	SpiInit(); //硬件SPI初始化
	Init_PWM(); //初始化PWM
  delay_ms(200);//初始化延时	
}

