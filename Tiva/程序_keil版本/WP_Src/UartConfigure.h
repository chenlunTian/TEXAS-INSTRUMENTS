/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: UartConfigure.h
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
********************************End of Head************************************/

 
#ifndef __UART_CONFIGURE_H_
#define __UART_CONFIGURE_H_
#include "Ringbuf.h"

void ConfigureUART0(void);
void ConfigureUART1(void);
void ConfigureUART2(unsigned long bound);
void ConfigureUART3(void);
void ConfigureUART6(void);
void ConfigureUART7(void);
void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void wust_sendware(unsigned char *wareaddr, int16_t waresize);
void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void Vcan_Send(void);
extern RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;
#endif


 
/********************************End of File************************************/


