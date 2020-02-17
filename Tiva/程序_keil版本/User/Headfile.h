/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: Headfile.h
**
** 创建人员: CLTian
**
** 创建日期: 2019-07-21
**
** 文档描述: 
**
**----------------------------------版本信息------------------------------------
** 版本代号: V0.1
**
** 版本说明: 初始版本
**
**------------------------------------------------------------------------------
\********************************End of Head************************************/


#ifndef __HEADFILE_H__
#define __HEADFILE_H__




#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>



/***********************************
重定义数据类型
************************************/
typedef   signed           char int8;
typedef unsigned           char u8;
typedef unsigned           char uint8;
typedef unsigned           char byte;
typedef   signed short     int int16;
typedef unsigned short     int uint16;
typedef unsigned short     int u16;
typedef unsigned long     int u32; 

extern	unsigned int  SysClock;
/***********************************
设置中断优先级
************************************/
#define  USER_INT0  0x00   //INTERRUPT
#define  USER_INT1  0x20   //UART2
#define  USER_INT2  0x40   //UART0
#define  USER_INT3  0x60   //UART3
#define  USER_INT4  0x80   //UART1
#define  USER_INT5  0xA0   //UART6
#define  USER_INT6  0xD0   //UART7
#define  USER_INT7  0xE0   //TIMER0

/***********************************
串口环形数据缓冲区数据长度宏定义
************************************/
#define UART7_BUF_CNT  4
#define UART6_BUF_CNT  28
#define UART3_BUF_CNT  24
#define UART2_BUF_CNT  200
#define UART1_BUF_CNT  50
#define UART0_BUF_CNT  32
/***********************************
声明系统库函数头文件
************************************/
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "pwm.h"
#include "rom.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"
#include "eeprom.h"
#include "ssi.h"
#include "hw_ssi.h"

/***********************************
声明自己写的库头文件
************************************/
#include "TimerConfigure.h"
#include "Time.h"
#include "UartConfigure.h"
#include "uartstdio.h"
#include "ADCConfigure.h"
#include "InterruptConfigure.h"
#include "myiic.h"
#include "LED.h"
#include "Key.h"
#include "Time.h"
#include "systems.h"
#include "SPI.h"
#include "PWMConfigure.h"

#endif

/********************************End of File************************************/
