/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: LED.c
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
#include "LED.h"

/***************************************************
������: void LED_Init(void)
˵��:	LED״ָ̬ʾ�Ƴ�ʼ��
���:	��
����:	��
��ע:	�ϵ��ʼ��������һ��
****************************************************/
void LED_Init()
{
	//ʹ��GPIOF��GPIOC������ʱ��
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	// ����PC2������commitλ
	HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL)   |= GPIO_PIN_2;
	HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = 0x0;
	//��PC2��PF1��PF2��PF3��Ϊ���
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2 |GPIO_PIN_3);
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_2);
}
void LED()
{
	    	//��PF1��PF2��PF3�øߣ�����3��LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    	delay(500);//Լ��ʱ500ms
    	//��PF1��PF2��PF3�õͣ�Ϩ��3��LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    	delay(500);//Լ��ʱ500ms
}
void LED1()
{
	    	//��PF1��PF2��PF3�øߣ�����3��LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
    	delay(500);//Լ��ʱ500ms
    	//��PF1��PF2��PF3�õͣ�Ϩ��3��LED
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    	delay(500);//Լ��ʱ500ms
}
