/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: TimerConfigure.c
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
#include "TimerConfigure.h"

/*******************************************************************************
** 函数名称: Time_init
** 功能描述: 系统调度定时器初始化
** 参数说明: None
** 返回说明: None
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Time_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);							//定时器0时钟外设使能				
  TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);						//32位周期定时器				
  TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/200);		//设定装载值,（80M/200）*1/80M=5ms				
  IntEnable(INT_TIMER0A);																		//总中断使能				
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); 					//中断输出, 设置模式;			
  TimerIntRegister(TIMER0_BASE,TIMER_A,TIMER0A_Handler);		//中断函数注册
  IntMasterEnable();			
  TimerEnable(TIMER0_BASE,TIMER_A); 												//定时器使能开始计数
  IntPrioritySet(INT_TIMER0A,USER_INT7);
}

/*******************************************************************************
** 函数名称: TIMER0A_Handler
** 功能描述: 定时器0中断函数
** 参数说明: None
** 返回说明: None
** 创建人员: CLTian
** 创建日期: 2019-07-22
********************************************************************************/

void TIMER0A_Handler(void)				//中断函数
{
	//你的任务		
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);	//清除中断标志位
}
