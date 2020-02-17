/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: ADCConfigure.c
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
#include "ADCConfigure.h"
#include "hw_adc.h"
#include "adc.h"

uint32_t value[8];
double value_filter;

/*******************************************************************************
** 函数名称: ADC_Init
** 功能描述: 电压测量端口PE3，作为ADC0的通道0初始化，非中断式采集
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void ADC_Init(void)//ADC初始化配置   
{    
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// 使能ADC1外设时钟.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));// 等待ADC1外设时钟准备就绪	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //使能GPIOE   
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //配置PE3为ADC输入
  //ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
  //ADC0的通道0初始化，非中断式采集
  //通过 ADCProcessorTrigger()函数生成的触发器   
  ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0); 
  //ADCHardwareOversampleConfigure(ADC0_BASE, 8);	
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 |ADC_CTL_END | ADC_CTL_IE);    
	//ADCIntClear(ADC0_BASE, 0);
	ADCSequenceEnable(ADC0_BASE, 0);    
  //ADCIntEnable(ADC0_BASE, 0);   
} 


float Battery_Voltage;
/*******************************************************************************
** 函数名称: Get_Battery_Voltage
** 功能描述: ADC获取 
** 参数说明: None
** 返回说明: None
** 描    述: 测量电池电压，飞控默认分压比位11，故测量电压不要超过
**           3.3V*11=36.6V，若想测更大的电压，调整板子上分压电阻阻值即可 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Get_Battery_Voltage(void)//ADC获取   
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

