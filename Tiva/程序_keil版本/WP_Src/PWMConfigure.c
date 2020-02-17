/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: PWMConfigure.c
**
** ������Ա: CLTian
**
** ��������: 2019-07-25
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
#include "PWMConfigure.h"
#define PWM_Period_MAX   3125 //2.5ms��������400hz��������
static uint16_t period;

/*******************************************************************************
** ��������: Init_PWM
** ��������: PWM��ʼ��
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-25
**------------------------------------------------------------------------------
********************************************************************************/

void Init_PWM(void)
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Set divider to 80M/8=10M=0.1us
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PC4_M0PWM6);
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);//M0PWM6�ɷ�����3���Ƽ�PWM_GEN_3
  // ����PWM���¼�������ʹ����
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  // ��������Ϊ2.5���루400���ȣ�
  period = PWM_Period_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT
                 | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}

/*******************************************************************************
** ��������: PWM_Output
** ��������: pwm���
** ����˵��: width1pwm����
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-25
**------------------------------------------------------------------------------
********************************************************************************/

void PWM_Output(uint16_t width1)
{
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,width1);//PC4 

}
