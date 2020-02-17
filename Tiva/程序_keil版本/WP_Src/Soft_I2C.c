/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: Soft_I2C.c
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
#include "Soft_I2C.h"


#define USE_SOFT_I2C 1  //���ʹ��Ӳ��I2C0ʱ��ʹ��0

#define SYSCTL_PERIPH_GPIO_I2C SYSCTL_PERIPH_GPIOB
#define GPIO_I2C   		GPIO_PORTB_BASE 
#define SCL_PIN   		GPIO_PIN_2
#define SDA_PIN   		GPIO_PIN_3
#define I2C_READ_SDA  GPIOPinRead(GPIO_I2C,SDA_PIN)			//SDA 
#define I2C_SDA_H  		GPIOPinWrite(GPIO_I2C,SDA_PIN,SDA_PIN)//SDA 
#define I2C_SDA_L 		GPIOPinWrite(GPIO_I2C,SDA_PIN,0)			//SDA
#define I2C_SCL_H  		GPIOPinWrite(GPIO_I2C,SCL_PIN,SCL_PIN)//SCL
#define I2C_SCL_L  		GPIOPinWrite(GPIO_I2C,SCL_PIN,0)			//SCL


void I2C_GPIO_Config(void)
{
#if (USE_SOFT_I2C)	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO_I2C);
	
	HWREG(GPIO_I2C + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����
	HWREG(GPIO_I2C + GPIO_O_CR) |= 0x000000FF;//ȷ��
	HWREG(GPIO_I2C + GPIO_O_LOCK) = 0;//��������
	
  GPIOPinTypeGPIOOutput(GPIO_I2C, SCL_PIN);
  GPIOPinTypeGPIOOutput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SCL_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
  Delay_Ms(10);
  GPIOPinWrite(GPIO_I2C, SCL_PIN, SCL_PIN);//���ø�
  GPIOPinWrite(GPIO_I2C, SDA_PIN, SDA_PIN);//���ø�
#else
  Init_I2C0();
#endif	
}

#if (USE_SOFT_I2C)
void I2C_SDA_OUT(void)
{
	GPIOPinTypeGPIOOutput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void I2C_SDA_IN(void)
{
  GPIOPinTypeGPIOInput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

#define SDA_OUT I2C_SDA_OUT()
#define SDA_IN  I2C_SDA_IN()
#define sdaRead I2C_READ_SDA  
#define SDA_H   I2C_SDA_H  		 
#define SDA_L   I2C_SDA_L
#define SCL_H   I2C_SCL_H  		
#define SCL_L   I2C_SCL_L  		


static void i2cDelay()
{
//    volatile int i = 7;//7
//    while (i)
//    i--;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA�����½���Ϊ��ʼ�ź�
static bool i2cStart()
{
    SDA_OUT;
    SCL_H;
    SDA_H;
    i2cDelay();
    if (!sdaRead)  // ���SDAΪ�͵�ƽ��������æ���˳�
        return false;
    SDA_L;
    if (sdaRead)  // ���SDAΪ�ߵ�ƽ��������æ���˳�
        return false;
    SDA_L;
    return true;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA����������Ϊֹͣ�ź�
static void i2cStop(void)
{
    SDA_OUT;
    SCL_L; 
    SDA_L;
    i2cDelay();  // STOP:when CLK is high DATA from low to high 
    SCL_H;
    SDA_H;  
    i2cDelay();
}
 
static void i2cAck(void)
{
    SDA_OUT;
    SCL_L;
    i2cDelay();
    SDA_L;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SCL_L;
}
 
static void i2cNoAck(void)
{
    SDA_OUT;
    SCL_L;
    i2cDelay();
    SDA_H;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SCL_L;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA��ƽ�����豸���ͱ�ʾӦ��
static bool i2cWaitAck(void)
{
    uint8_t errTimes = 0;
    SDA_IN;
    SDA_H;
    i2cDelay();
    SCL_H;
    i2cDelay();
    while (sdaRead) {
        if (errTimes++ > 20) {
            SCL_L;
            return false;
        }           
        i2cDelay();
    }
    SCL_L;
    return true;
}
 
// �������ݣ����ݴӸ�λ����λ����  
static void i2cSendByte(uint8_t byte)  
{
    uint8_t i = 8;
 
    SDA_OUT;
    while (i--) {      
        SCL_L;  // ʱ���ź�Ϊ�͵�ƽ�ڼ䣬���������ߵ�ƽ�仯
        i2cDelay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L; 
        byte <<= 1; 
        i2cDelay();
        SCL_H;
        i2cDelay();
    }
    SCL_L;
}
 
static uint8_t i2cReceiveByte()  
{
    uint8_t i = 8;
    uint8_t byte = 0;
    SDA_IN;
    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_H;
        i2cDelay();
        if (sdaRead) {
            byte |= 0x01;
        }
        SCL_L;
        i2cDelay();
    }
    SCL_L;
    return byte; 
}
 
 

 
/**
 * ͨ��I2C����дһ�ֽ�����
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] data:Ҫд�������
 */
bool i2cWriteOneByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);  // �ӻ���ַ�ɸ�7λ+��дλ����   
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);       
    i2cWaitAck();
    i2cSendByte(data);     
    i2cWaitAck();
    return true;
}
 

/**
 *  
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�ֽ��� 
 * @param[in] data:��д������� 
 */
uint8_t i2cReadOneBytes(uint8_t dev, uint8_t reg)
{
	unsigned char REG_data;
  i2cStart();
  i2cSendByte(dev<<1);
	i2cWaitAck();
  i2cSendByte(reg);
	i2cWaitAck();
  i2cStart();
  i2cSendByte((dev << 1) | 0x01);
  i2cWaitAck();
	REG_data=i2cReceiveByte();
  i2cStop();
  return REG_data;	
} 


/**
 *  
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�ֽ��� 
 * @param[in] data:��д������� 
 */
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;
 
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);          
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(dev);   
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (!i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}
 
 
/**
 * ��I2C�豸�ж�ȡ����
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�����ֽ���
 * @param[out] data:����������
 */
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);      
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);     
    i2cWaitAck();
    i2cStart();           
    i2cSendByte((dev << 1) | 0x01);  // ������ַ+������    
    i2cWaitAck();
    while (len) {
        *data = i2cReceiveByte();
        if (len == 1)
            i2cNoAck();  // ���һ���ֽڲ�Ӧ��
        else
            i2cAck();
        data++;
        len--;
    }
    i2cStop();
    return true;
}
#endif


void Single_WriteI2C0(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
#if (USE_SOFT_I2C)
  i2cWriteOneByte(SlaveAddress,REG_Address,REG_data);
#else
  i2c0Write(SlaveAddress,REG_Address,REG_data);
#endif
}

//**************************************
unsigned char Single_ReadI2C0(unsigned char SlaveAddress,unsigned char REG_Address)
{
#if (USE_SOFT_I2C)
  return i2cReadOneBytes(SlaveAddress,REG_Address);
#else
  return i2c0Read(SlaveAddress,REG_Address);
#endif
}

