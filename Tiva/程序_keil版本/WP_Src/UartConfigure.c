/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: UartConfigure.c
**
** ������Ա: CLTian
**
** ��������: 2019-07-22
**
** �ĵ�����: 
**
**----------------------------------�汾��Ϣ------------------------------------
** �汾����: V0.1
**
** �汾˵��: ��ʼ�汾
**
**------------------------------------------------------------------------------
*******************************************************************************/

#include "Headfile.h"
#include "uart.h"
#include "UartConfigure.h"
#include "Ringbuf.h"
//����ѭ�����л������ݶ���
RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;

/*******************************************************************************
** ��������: UART0_IRQHandler
** ��������: ����0���ݽ���
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART0_IRQHandler(void)//UART0�жϺ���
{	
  //��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //����жϱ�־	
  UARTIntClear(UART0_BASE,flag);		
  //�ж�FIFO�Ƿ�������		
  while(UARTCharsAvail(UART0_BASE))		
  {			
    RingBuf_Write(UARTCharGet(UART0_BASE),&COM0_Rx_Buf,UART0_BUF_CNT);//�����ζ�������д����	
  }
}

/*******************************************************************************
** ��������: ConfigureUART0
** ��������: ����0����
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART0(void)//����0��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//ʹ��UART����
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIOģʽ���� PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
  //UARTЭ������ ������115200 8λ 1ֹͣλ  ��У��λ	
  //UART����FIFO Ĭ��FIFO LevelΪ4/8 �Ĵ�����8�ֽں�����ж�	//���ú����1λ�Ͳ����ж�	
  UARTFIFODisable(UART0_BASE);//ʹ��UART0�ж�	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART0, USER_INT2);
}

/*******************************************************************************
** ��������: USART0_Send
** ��������: ����0����N���ֽ�����
** ����˵��: pui8Buffer: [����/��] 
**			 ui32Count: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** ��������: wust_sendware
** ��������: ɽ������ʾ��������
** ����˵��: wareaddr: [����/��] 
**			 waresize: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void wust_sendware(unsigned char *wareaddr, int16_t waresize)//ɽ�ⷢ�Ͳ���
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//֡ͷ
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//֡β
  USART0_Send(cmdf, sizeof(cmdf));
  USART0_Send(wareaddr, waresize);
  USART0_Send(cmdr, sizeof(cmdr));
}


/*******************************************************************************
** ��������: UART1_IRQHandler
** ��������: ����1���ݽ���
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART1_IRQHandler(void)//UART1�жϺ���
{				
  uint32_t flag = UARTIntStatus(UART1_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART1_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART1_BASE))//�ж�FIFO�Ƿ�������		
  {			
    RingBuf_Write(UARTCharGet(UART1_BASE),&COM1_Rx_Buf,UART1_BUF_CNT);//�����ζ�������д����	
		
  }
}



/*******************************************************************************
** ��������: USART1_Send
** ��������: ����1����N���ֽ�����
** ����˵��: pui8Buffer: [����/��] 
**			 ui32Count: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** ��������: ConfigureUART1
** ��������: ����1����
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART1(void)//����1��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//ʹ��UART����
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIOģʽ���� PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(1, 115200, 16000000);
  UARTFIFODisable(UART1_BASE);//ʹ��UART1�ж�	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//ʹ��UART1�����ж�		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1�жϵ�ַע��	
  IntPrioritySet(INT_UART1, USER_INT4); //���ô���1�ж����ȼ�
}


/*******************************************************************************
** ��������: UART2_IRQHandler
** ��������: ����2���ݽ���
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART2_IRQHandler(void)
{
  uint32_t flag = UARTIntStatus(UART2_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART2_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART2_BASE))//�ж�FIFO�Ƿ�������				
  {		
    //����õ�������
    RingBuf_Write(UARTCharGet(UART2_BASE),&COM2_Rx_Buf,UART2_BUF_CNT);//�����ζ�������д����
    if(COM2_Rx_Buf.Ring_Buff[0]!=0XB5)
    {
      COM2_Rx_Buf.Head=1;
      COM2_Rx_Buf.Tail=0; 
    }		
  }
}

/*******************************************************************************
** ��������: USART2_Send
** ��������: ����2����N���ֽ�����
** ����˵��: pui8Buffer: [����/��] 
**			 ui32Count: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART2_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** ��������: ConfigureUART2
** ��������: ����2����
** ����˵��: bound: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART2(unsigned long bound)//����2��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//ʹ��UART����
  
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����PD6
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//ȷ��
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//��������
  
  GPIOPinConfigure(GPIO_PD6_U2RX);//GPIOģʽ���� PD6--RX PD7--TX 
  GPIOPinConfigure(GPIO_PD7_U2TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART2_BASE);//ʹ��UART2�ж�	
  UARTIntEnable(UART2_BASE,UART_INT_RX |UART_INT_RT);//ʹ��UART0�����ж�		
  UARTIntRegister(UART2_BASE,UART2_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART2, USER_INT1);
}

/*******************************************************************************
** ��������: UART3_IRQHandler
** ��������: ����3���ݽ���
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART3_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART3_BASE,flag);//����жϱ�־			
  while(UARTCharsAvail(UART3_BASE))//�ж�FIFO�Ƿ�������		
  {			
    RingBuf_Write(UARTCharGet(UART3_BASE),&COM3_Rx_Buf,UART3_BUF_CNT);//�����ζ�������д����		
  }
}

/*******************************************************************************
** ��������: USART3_Send
** ��������: ����3����N���ֽ�����
** ����˵��: pui8Buffer: [����/��] 
**			 ui32Count: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}


/*******************************************************************************
** ��������: ConfigureUART3
** ��������: ����3����
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART3(void)//����3��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//ʹ��UART����
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIOģʽ���� PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//ʹ��UART3�����ж�		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3�жϵ�ַע��	
  IntPrioritySet(INT_UART3, USER_INT3);
}

/*******************************************************************************
** ��������: UART6_IRQHandler
** ��������: ����6���ݽ���
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART6_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART6_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART6_BASE))//�ж�FIFO�Ƿ�������		
  {			
    RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_BUF_CNT);//�����ζ�������д����		
  }
}

/*******************************************************************************
** ��������: USART6_Send
** ��������: ����6����N���ֽ�����
** ����˵��: pui8Buffer: [����/��] 
**			 ui32Count: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** ��������: ConfigureUART6
** ��������: ����6����
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART6(void)//����6��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//ʹ��UART����
  GPIOPinConfigure(GPIO_PD4_U6RX);//GPIOģʽ���� PD4--RX PD5--TX 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 19200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//ʹ��UART6�����ж�		
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART6�жϵ�ַע��	
  IntPrioritySet(INT_UART6, USER_INT5);
}

/*******************************************************************************
** ��������: UART7_IRQHandler
** ��������: ����7���ݽ���
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void UART7_IRQHandler(void)//UART2�жϺ���
{		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART7_BASE,flag);//����жϱ�־		
  while(UARTCharsAvail(UART7_BASE))//�ж�FIFO�Ƿ�������			
  {			
    //����õ�������			
    //UARTCharPut(UART1_BASE,UARTCharGet(UART1_BASE));
    RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,UART7_BUF_CNT);//�����ζ�������д����		
  }
}

/*******************************************************************************
** ��������: USART7_Send
** ��������: ����7����N���ֽ�����
** ����˵��: pui8Buffer: [����/��] 
**			 ui32Count: [����/��] 
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void USART7_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART7_BASE, *pui8Buffer++);
  }
}

/*******************************************************************************
** ��������: ConfigureUART7
** ��������: ����7����
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ConfigureUART7(void)//����7��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//ʹ��UART����
  GPIOPinConfigure(GPIO_PE0_U7RX);//GPIOģʽ���� PE0--RX PE1--TX 
  GPIOPinConfigure(GPIO_PE1_U7TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART7_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART7_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART7, USER_INT6);
}



/*******************************************************************************
** ��������: Vcan_Send
** ��������: ɽ�����ݷ��ͺ�����Ĭ�Ϸ���8��ͨ������������Ϊfloat
��ÿ��ͨ�����ݿ����Լ�����
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Vcan_Send(void)//ɽ�����վ����
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


