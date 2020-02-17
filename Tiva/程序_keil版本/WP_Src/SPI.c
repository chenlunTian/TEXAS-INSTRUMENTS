/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: SPI.c
**
** 创建人员: CLTian
**
** 创建日期: 2019-07-24
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
#include "SPI.h"

void SpiInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    /* 设置SSI IO口的模式 */
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

    /* IO口配置为SSI功能 */
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    /* SSI配置 模式3(Polarity = 1 Phase = 1) 主设备模式 速率1MHz 数据长度8位*/
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8);
    /* 使能SSI2 */
    SSIEnable(SSI2_BASE);
}

/* SPI读写函数 */
/*******************************************************************************
** 函数名称: SPI_RW
** 功能描述: 
** 参数说明: SendData读写的数据
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-25
**------------------------------------------------------------------------------
********************************************************************************/

uint8_t SPI_RW(uint8_t SendData)
{
    uint32_t TempData;
    uint8_t ReceiveData;

    /* 向SSI FIFO写入数据 */
    SSIDataPutNonBlocking(SSI2_BASE, SendData);
    /* 等待SSI不忙 */
    while(SSIBusy(SSI2_BASE));
    /* 从FIFO读取数据 */
    SSIDataGetNonBlocking(SSI2_BASE, &TempData);

    /* 截取数据的低八位 */
    ReceiveData = TempData & 0xff;

    return ReceiveData;
}

/*******************************************************************************
** 函数名称: SPITransmit
** 功能描述: spi连续读取
** 参数说明: Data读取的数据存放的指针 
**			 Size数据大小 
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-25
**------------------------------------------------------------------------------
********************************************************************************/

void SPITransmit(uint8_t *Data, uint16_t Size)
{
    uint16_t i = 0;
    /* 连续写入数据 */
    for(i = 0; i < Size; i++)
    {
        SPI_RW(Data[i]);
    }
}

/*******************************************************************************
** 函数名称: SPIReceive
** 功能描述: SPI连续写入
** 参数说明: Data写入数据存放的指针
**			 Size数据大小
** 返回说明: None
** 描    述:  
** 创建人员: CLTian
** 创建日期: 2019-07-25
**------------------------------------------------------------------------------
********************************************************************************/

void SPIReceive(uint8_t *Data, uint16_t Size)
{
    uint16_t i = 0;
    /* 连续读取数据 */
    for(i = 0; i < Size; i++)
    {
        Data[i] = SPI_RW(0xFF);
    }
}
