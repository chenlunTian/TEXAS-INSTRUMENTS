
#include "Headfile.h"
#include "Key.h"

/*******************************************************************************
** ��������: Key_Init
** ��������: ������ʼ��
** ����˵��: None
** ����˵��: None
** ��    ��:  
** ������Ա: CLTian
** ��������: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/

void Key_Init(void)
{
	//ʹ��GPIOF��GPIOE������ʱ��
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//PF4������������commitλ
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_4;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0x0;
	//����GPIOE2��GPIOF4Ϊ����,2MA����������������
  GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_DIR_MODE_IN);//SW1
  GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_DIR_MODE_IN);//SW2
  GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

/*******************************************************************************
** ��������: Key_Scan
** ��������: ����ɨ�裬
** ����˵��: release
** ����˵��: bool
** ��    ��: ��ڲ���release�����Ƿ񿪷Ű���ɨ��Ȩ��
**           ����ɨ�践��TRUE����������ʱΪ�͵�ƽ���ͷź�IO���õ�������
**           ����ģʽ������Ϊ�ߵ�ƽ 
** ������Ա: CLTian
** ��������: 2019-07-23
**------------------------------------------------------------------------------
********************************************************************************/

bool Key_Scan(uint8_t release)
{
  if(release==1)  return false;
  if(QuadKey1==0)
  {
    delay_ms(10);//��ʱȥ��
    if(QuadKey1==0)
    {
      while(QuadKey1==0);//�ȴ������ͷ�
		//��ĺ���
    }
  }
  
  if(QuadKey2==0)
  {
    delay_ms(10);//��ʱȥ��
    if(QuadKey2==0)
    {
      while(QuadKey2==0);//�ȴ������ͷ�
			//��ĺ���
    }
  }
  return true;
}
