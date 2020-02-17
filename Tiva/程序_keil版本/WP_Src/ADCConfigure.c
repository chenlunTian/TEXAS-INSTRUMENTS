/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: ADCConfigure.c
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
#include "ADCConfigure.h"
#include "hw_adc.h"
#include "adc.h"

uint32_t value[8];
double value_filter;

/*******************************************************************************
** ��������: ADC_Init
** ��������: ��ѹ�����˿�PE3����ΪADC0��ͨ��0��ʼ�������ж�ʽ�ɼ�
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ADC_Init(void)//ADC��ʼ������   
{    
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// ʹ��ADC1����ʱ��.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));// �ȴ�ADC1����ʱ��׼������	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //ʹ��GPIOE   
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //����PE3ΪADC����
  //ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
  //ADC0��ͨ��0��ʼ�������ж�ʽ�ɼ�
  //ͨ�� ADCProcessorTrigger()�������ɵĴ�����   
  ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0); 
  //ADCHardwareOversampleConfigure(ADC0_BASE, 8);	
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 |ADC_CTL_END | ADC_CTL_IE);    
	//ADCIntClear(ADC0_BASE, 0);
	ADCSequenceEnable(ADC0_BASE, 0);    
  //ADCIntEnable(ADC0_BASE, 0);   
} 


float Battery_Voltage;
/*******************************************************************************
** ��������: Get_Battery_Voltage
** ��������: ADC��ȡ 
** ����˵��: None
** ����˵��: None
** ��    ��: ������ص�ѹ���ɿ�Ĭ�Ϸ�ѹ��λ11���ʲ�����ѹ��Ҫ����
**           3.3V*11=36.6V����������ĵ�ѹ�����������Ϸ�ѹ������ֵ���� 
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Get_Battery_Voltage(void)//ADC��ȡ   
{
	ADCProcessorTrigger(ADC0_BASE, 0);   
	while(!ADCIntStatus(ADC0_BASE, 0, false)) {;}
	ADCIntClear(ADC0_BASE, 0);	
	ADCSequenceDataGet(ADC0_BASE, 0, value);   
	//value[0] =  HWREG(ADC0_BASE+ ADC_SEQ + (ADC_SEQ_STEP*0) + ADC_SSFIFO);
	value_filter=(double)(0.7*value_filter+10.89*value[0]/4095.0f);
	Battery_Voltage=value_filter;		
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

