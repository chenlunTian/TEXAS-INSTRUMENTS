/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: LED.c
**
** 创建人员: CLTian
**
** 创建日期: 2019-07-22
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
#include "LED.h"

/***************************************************
函数名: void LED_Init(void)
说明:	LED状态指示灯初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void LED_Init()
{
	//使能GPIOF和GPIOC的外设时钟
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	// 解锁PC2并设置commit位
	HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL)   |= GPIO_PIN_2;
	HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = 0x0;
	//将PC2、PF1、PF2和PF3设为输出
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2 |GPIO_PIN_3);
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_2);
}
void LED()
{
	    	//将PF1、PF2和PF3置高，点亮3个LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    	delay(500);//约延时500ms
    	//将PF1、PF2和PF3置低，熄灭3个LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    	delay(500);//约延时500ms
}
void LED1()
{
	    	//将PF1、PF2和PF3置高，点亮3个LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
    	delay(500);//约延时500ms
    	//将PF1、PF2和PF3置低，熄灭3个LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    	delay(500);//约延时500ms
}
