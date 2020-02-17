/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: Time.c
**
** 创建人员: CLTian
**
** 创建日期: 2019-07-22
**
** 文档描述: 系统滴答
**
**----------------------------------版本信息------------------------------------
** 版本代号: V0.1
**
** 版本说明: 初始版本
**
**------------------------------------------------------------------------------
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Time.h"

#include "sysctl.h"
#include "systick.h"

static volatile uint32_t counter;

static void SycTickHandler(void) {
  counter++;
}

void initTime(void) {
  SysTickPeriodSet(SysCtlClockGet() / 1000000UL); // 1000 for milliseconds & 1000000 for microseconds
  SysTickIntRegister(SycTickHandler);
  SysTickIntEnable();
  SysTickEnable();
}

void delay(uint32_t ms) {
  delayMicroseconds(ms * 1000UL);
}

void delayMicroseconds(uint32_t us) {
  uint32_t start = micros();
  while ((int32_t)(micros() - start) < us) {
    // Do nothing
  };
}

uint32_t millis(void) {
  return counter / 1000UL;
}

uint32_t micros(void) {
  return counter;
}

void Delay_Ms(uint32_t x)
{
  delay(x);
}

void delay_ms(uint32_t x)
{
  Delay_Ms(x);
}


void delay_us(uint32_t x)
{
  delayMicroseconds(x);
}


void Delay_Us(uint32_t x) 
{
  delayMicroseconds(x);
}
