
#include "Headfile.h"
#include "Key.h"

/*******************************************************************************
** 函数名称: Key_Init
** 功能描述: 按键初始化
** 参数说明: None
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Key_Init(void)
{
	//使能GPIOF和GPIOE的外设时钟
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//PF4解锁，并设置commit位
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_4;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0x0;
	//设置GPIOE2、GPIOF4为输入,2MA驱动能力、弱上拉
  GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_DIR_MODE_IN);//SW1
  GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_DIR_MODE_IN);//SW2
  GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

/*******************************************************************************
** 函数名称: Key_Scan
** 功能描述: 按键扫描，
** 参数说明: release
** 返回说明: bool
** 描    述: 入口参数release决定是否开放按键扫描权限
**           正常扫描返回TRUE，按键按下时为低电平，释放后IO配置的是上拉
**           输入模式，悬空为高电平 
** 创建人员: CLTian
** 创建日期: 2019-07-23
**------------------------------------------------------------------------------
********************************************************************************/

bool Key_Scan(uint8_t release)
{
  if(release==1)  return false;
  if(QuadKey1==0)
  {
    delay_ms(10);//延时去抖
    if(QuadKey1==0)
    {
      while(QuadKey1==0);//等待按键释放
		//你的函数
    }
  }
  
  if(QuadKey2==0)
  {
    delay_ms(10);//延时去抖
    if(QuadKey2==0)
    {
      while(QuadKey2==0);//等待按键释放
			//你的函数
    }
  }
  return true;
}
