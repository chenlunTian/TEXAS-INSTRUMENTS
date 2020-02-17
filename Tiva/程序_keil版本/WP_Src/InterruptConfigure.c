/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: InterruptConfigure.c
**
** 创建人员: CLTian
**
** 创建日期: 2019-07-25
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
#include "InterruptConfigure.h"

/* 声明中断服务函数，TM4C的中断服务函数名可以自己定义的 */
/* GPIOF中断服务函数 */
void GPIO_PortFIntHandler(void)
{
		//你的函数
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}

void InterruptInit(void)
{
    /* 使能GPIOF */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /* PF4方向为输入 */
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
    /* 配置为上拉模式 */
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* 注册GPIOF的中断服务函数 */
    GPIOIntRegister(GPIO_PORTF_BASE, GPIO_PortFIntHandler);
    /* 设置PF4为下降沿触发 */
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    /* 使能PF4中断 */
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    /* 使能GPIOF中断 */
    IntEnable(INT_GPIOF);
    /* 设置中断优先级，TM4C123G的中断优先级有8个 */
    IntPrioritySet(INT_GPIOF, USER_INT0);
    /* 清除PF4中断标志位 */
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}



