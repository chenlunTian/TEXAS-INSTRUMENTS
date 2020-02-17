/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: systick.c
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
//*****************************************************************************
//
// systick.c - Driver for the SysTick timer in NVIC.
//
// Copyright (c) 2005-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.0.1.11577 of the Tiva Peripheral Driver Library.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup systick_api
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_types.h"
#include "debug.h"
#include "interrupt.h"
#include "systick.h"



/*******************************************************************************
** 函数名称: SysTickEnable
** 功能描述: 使能SysTick计数器
** 参数说明: None
** 返回说明: None
** 描    述: 该函数启动SysTick计数器。如果一个中断处理程序已经声明，
**						那么当SysTick计数器计数溢出时，调用该函数。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
SysTickEnable(void)
{
	//
	// Enable SysTick.
	//
	HWREG(NVIC_ST_CTRL) |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
}


/*******************************************************************************
** 函数名称: SysTickDisable
** 功能描述: 禁用SysTick计数器
** 参数说明: None
** 返回说明: None
** 描    述:  此功能停止SysTick计数器。如果中断处理程序已注册，只有重新启动SysTick才能调用。
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
SysTickDisable(void)
{
	//
	// Disable SysTick.
	//
	HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_ENABLE);
}


/*******************************************************************************
** 函数名称: SysTickIntRegister
** 功能描述: SysTick中断注册中断处理程序
** 参数说明: pfnhandler是一个指针，指向当SysTick中断发生时要调用的函数。
** 返回说明: None
** 描    述: 此函数注册在发生SysTick中断时要调用的处理程序。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
SysTickIntRegister(void (*pfnHandler)(void))
{
	//
	// Register the interrupt handler, returning an error if an error occurs.
	//
	IntRegister(FAULT_SYSTICK, pfnHandler);

	//
	// Enable the SysTick interrupt.
	//
	HWREG(NVIC_ST_CTRL) |= NVIC_ST_CTRL_INTEN;
}



/*******************************************************************************
** 函数名称: SysTickIntUnregister
** 功能描述: 注销SysTick中断的中断处理程序。
** 参数说明: None
** 返回说明: None
** 描    述: 此函数注销在发生SysTick中断时要调用的处理程序。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
SysTickIntUnregister(void)
{
	//
	// Disable the SysTick interrupt.
	//
	HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);

	//
	// Unregister the interrupt handler.
	//
	IntUnregister(FAULT_SYSTICK);
}



/*******************************************************************************
** 函数名称: SysTickIntEnable
** 功能描述: 使能SysTick中断
** 参数说明: None
** 返回说明: None
** 描    述: 该函数使能SysTick中断，允许它被反映到处理器
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
SysTickIntEnable(void)
{
	//
	// Enable the SysTick interrupt.
	//
	HWREG(NVIC_ST_CTRL) |= NVIC_ST_CTRL_INTEN;
}


void
/*******************************************************************************
** 函数名称: SysTickIntDisable
** 功能描述: 禁用SysTick中断
** 参数说明: None
** 返回说明: None
** 描    述: 该函数禁用SysTick中断，不允许它被反映到处理器 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

SysTickIntDisable(void)
{
	//
	// Disable the SysTick interrupt.
	//
	HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
}



/*******************************************************************************
** 函数名称: SysTickPeriodSet
** 功能描述: 设置SysTick计数器的周期
** 参数说明: ui32Period为每一个SysTick计数器周期的时钟数，必须在1~16777216之间
** 返回说明: None
** 描    述: 该函数设置SysTick计数器的频率，相当于两个中断之间的处理器时钟数
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void
SysTickPeriodSet(uint32_t ui32Period)
{
	//
	// Check the arguments.
	//
	ASSERT((ui32Period > 0) && (ui32Period <= 16777216));

	//
	// Set the period of the SysTick counter.
	//
	HWREG(NVIC_ST_RELOAD) = ui32Period - 1;
}


/*******************************************************************************
** 函数名称: SysTickPeriodGet
** 功能描述: 获取SysTick计数器的周期。
** 参数说明: None
** 返回说明: None
** 描    述: 此函数返回SysTick计数器频率，该频率等于中断之间处理器时钟的数目。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
uint32_t
SysTickPeriodGet(void)
{
	//
	// Return the period of the SysTick counter.
	//
	return(HWREG(NVIC_ST_RELOAD) + 1);
}

/*******************************************************************************
** 函数名称: SysTickValueGet
** 功能描述: 获取SysTick计数器的当前值
** 参数说明: None
** 返回说明: None
** 描    述: 此函数返回SysTick计数器的当前值，介于期间-1和零之间的值，包括。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
uint32_t
SysTickValueGet(void)
{
	//
	// Return the current value of the SysTick counter.
	//
	return(HWREG(NVIC_ST_CURRENT));
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
