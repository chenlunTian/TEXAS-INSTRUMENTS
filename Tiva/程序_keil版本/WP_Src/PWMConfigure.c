/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: PWMConfigure.c
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
#include "PWMConfigure.h"
#define PWM_Period_MAX   3125 //2.5ms————400hz设置周期
static uint16_t period;

/*******************************************************************************
** 函数名称: Init_PWM
** 功能描述: PWM初始化
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-25
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
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);//M0PWM6由发生器3控制即PWM_GEN_3
  // 配置PWM向下计数，即使更新
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  // 周期设置为2.5毫秒（400赫兹）
  period = PWM_Period_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT
                 | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}

/*******************************************************************************
** 函数名称: PWM_Output
** 功能描述: pwm输出
** 参数说明: width1pwm脉宽
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-25
**------------------------------------------------------------------------------
********************************************************************************/

void PWM_Output(uint16_t width1)
{
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,width1);//PC4 

}
