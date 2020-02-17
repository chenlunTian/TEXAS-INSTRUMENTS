/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: UartConfigure.c
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
#include "uart.h"
#include "UartConfigure.h"
#include "Ringbuf.h"
//串口循环队列缓冲数据定义
RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;

/*******************************************************************************
** 函数名称: UART0_IRQHandler
** 功能描述: 串口0数据接收
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART0_IRQHandler(void)//UART0中断函数
{	
  //获取中断标志 原始中断状态 不屏蔽中断标志		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //清除中断标志	
  UARTIntClear(UART0_BASE,flag);		
  //判断FIFO是否还有数据		
  while(UARTCharsAvail(UART0_BASE))		
  {			
    RingBuf_Write(UARTCharGet(UART0_BASE),&COM0_Rx_Buf,UART0_BUF_CNT);//往环形队列里面写数据	
  }
}

/*******************************************************************************
** 函数名称: ConfigureUART0
** 功能描述: 串口0配置
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART0(void)//串口0初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//使能UART外设
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIO模式配置 PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
  //UART协议配置 波特率115200 8位 1停止位  无校验位	
  //UART禁用FIFO 默认FIFO Level为4/8 寄存器满8字节后产生中断	//禁用后接收1位就产生中断	
  UARTFIFODisable(UART0_BASE);//使能UART0中断	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//使能UART0接收中断		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART0, USER_INT2);
}

/*******************************************************************************
** 函数名称: USART0_Send
** 功能描述: 串口0发送N个字节数据
** 参数说明: pui8Buffer: [输入/出] 
**			 ui32Count: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** 函数名称: wust_sendware
** 功能描述: 山外虚拟示波器发送
** 参数说明: wareaddr: [输入/出] 
**			 waresize: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void wust_sendware(unsigned char *wareaddr, int16_t waresize)//山外发送波形
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//帧头
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//帧尾
  USART0_Send(cmdf, sizeof(cmdf));
  USART0_Send(wareaddr, waresize);
  USART0_Send(cmdr, sizeof(cmdr));
}


/*******************************************************************************
** 函数名称: UART1_IRQHandler
** 功能描述: 串口1数据接收
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART1_IRQHandler(void)//UART1中断函数
{				
  uint32_t flag = UARTIntStatus(UART1_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART1_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART1_BASE))//判断FIFO是否还有数据		
  {			
    RingBuf_Write(UARTCharGet(UART1_BASE),&COM1_Rx_Buf,UART1_BUF_CNT);//往环形队列里面写数据	
		
  }
}



/*******************************************************************************
** 函数名称: USART1_Send
** 功能描述: 串口1发送N个字节数据
** 参数说明: pui8Buffer: [输入/出] 
**			 ui32Count: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** 函数名称: ConfigureUART1
** 功能描述: 串口1配置
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART1(void)//串口1初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//使能UART外设
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIO模式配置 PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
  UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(1, 115200, 16000000);
  UARTFIFODisable(UART1_BASE);//使能UART1中断	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//使能UART1接收中断		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1中断地址注册	
  IntPrioritySet(INT_UART1, USER_INT4); //设置串口1中断优先级
}


/*******************************************************************************
** 函数名称: UART2_IRQHandler
** 功能描述: 串口2数据接收
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART2_IRQHandler(void)
{
  uint32_t flag = UARTIntStatus(UART2_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART2_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART2_BASE))//判断FIFO是否还有数据				
  {		
    //输出得到的数据
    RingBuf_Write(UARTCharGet(UART2_BASE),&COM2_Rx_Buf,UART2_BUF_CNT);//往环形队列里面写数据
    if(COM2_Rx_Buf.Ring_Buff[0]!=0XB5)
    {
      COM2_Rx_Buf.Head=1;
      COM2_Rx_Buf.Tail=0; 
    }		
  }
}

/*******************************************************************************
** 函数名称: USART2_Send
** 功能描述: 串口2发送N个字节数据
** 参数说明: pui8Buffer: [输入/出] 
**			 ui32Count: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART2_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** 函数名称: ConfigureUART2
** 功能描述: 串口2配置
** 参数说明: bound: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART2(unsigned long bound)//串口2初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//使能UART外设
  
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁PD6
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//确认
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//重新锁定
  
  GPIOPinConfigure(GPIO_PD6_U2RX);//GPIO模式配置 PD6--RX PD7--TX 
  GPIOPinConfigure(GPIO_PD7_U2TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART2_BASE);//使能UART2中断	
  UARTIntEnable(UART2_BASE,UART_INT_RX |UART_INT_RT);//使能UART0接收中断		
  UARTIntRegister(UART2_BASE,UART2_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART2, USER_INT1);
}

/*******************************************************************************
** 函数名称: UART3_IRQHandler
** 功能描述: 串口3数据接收
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART3_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART3_BASE,flag);//清除中断标志			
  while(UARTCharsAvail(UART3_BASE))//判断FIFO是否还有数据		
  {			
    RingBuf_Write(UARTCharGet(UART3_BASE),&COM3_Rx_Buf,UART3_BUF_CNT);//往环形队列里面写数据		
  }
}

/*******************************************************************************
** 函数名称: USART3_Send
** 功能描述: 串口3发送N个字节数据
** 参数说明: pui8Buffer: [输入/出] 
**			 ui32Count: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}


/*******************************************************************************
** 函数名称: ConfigureUART3
** 功能描述: 串口3配置
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART3(void)//串口3初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//使能UART外设
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIO模式配置 PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//使能UART0中断	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//使能UART3接收中断		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3中断地址注册	
  IntPrioritySet(INT_UART3, USER_INT3);
}

/*******************************************************************************
** 函数名称: UART6_IRQHandler
** 功能描述: 串口6数据接收
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART6_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART6_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART6_BASE))//判断FIFO是否还有数据		
  {			
    RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_BUF_CNT);//往环形队列里面写数据		
  }
}

/*******************************************************************************
** 函数名称: USART6_Send
** 功能描述: 串口6发送N个字节数据
** 参数说明: pui8Buffer: [输入/出] 
**			 ui32Count: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** 函数名称: ConfigureUART6
** 功能描述: 串口6配置
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART6(void)//串口6初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//使能UART外设
  GPIOPinConfigure(GPIO_PD4_U6RX);//GPIO模式配置 PD4--RX PD5--TX 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 19200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//使能UART0中断	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//使能UART6接收中断		
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART6中断地址注册	
  IntPrioritySet(INT_UART6, USER_INT5);
}

/*******************************************************************************
** 函数名称: UART7_IRQHandler
** 功能描述: 串口7数据接收
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART7_IRQHandler(void)//UART2中断函数
{		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART7_BASE,flag);//清除中断标志		
  while(UARTCharsAvail(UART7_BASE))//判断FIFO是否还有数据			
  {			
    //输出得到的数据			
    //UARTCharPut(UART1_BASE,UARTCharGet(UART1_BASE));
    RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,UART7_BUF_CNT);//往环形队列里面写数据		
  }
}

/*******************************************************************************
** 函数名称: USART7_Send
** 功能描述: 串口7发送N个字节数据
** 参数说明: pui8Buffer: [输入/出] 
**			 ui32Count: [输入/出] 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART7_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART7_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** 函数名称: ConfigureUART7
** 功能描述: 串口7配置
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART7(void)//串口7初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//使能UART外设
  GPIOPinConfigure(GPIO_PE0_U7RX);//GPIO模式配置 PE0--RX PE1--TX 
  GPIOPinConfigure(GPIO_PE1_U7TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART7_BASE);//使能UART0中断	
  UARTIntEnable(UART7_BASE,UART_INT_RX);//使能UART0接收中断		
  UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART7, USER_INT6);
}



/*******************************************************************************
** 函数名称: Vcan_Send
** 功能描述: 山外数据发送函数，默认发送8个通道，数据类型为float
，每个通道数据可以自己定义
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Vcan_Send(void)//山外地面站发送
{
  static float DataBuf[8];	
/*	
	DataBuf[0]=0;
  DataBuf[1]=0;
  DataBuf[2]=0;
  DataBuf[3]=0;
  DataBuf[4]=0;
  DataBuf[5]=0;
  DataBuf[6]=0;
  DataBuf[7]=0;
*/
  wust_sendware((unsigned char *)DataBuf,sizeof(DataBuf));
}



/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/


