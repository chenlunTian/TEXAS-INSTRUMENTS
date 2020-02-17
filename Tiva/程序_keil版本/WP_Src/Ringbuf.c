/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------文件信息------------------------------------
** 文件名称: Ringbuf.c
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
#include "Ringbuf.h"



/*******************************************************************************
** 函数名称: RingBuff_Init
** 功能描述: 初始化环形缓冲区
** 参数说明: ringBuff: [输入/出] 
** 返回说明: None
** 描    述: 初始化环形缓冲区 
** 创建人员: CLTian
** 创建日期: 2019-08-21
**------------------------------------------------------------------------------
********************************************************************************/

void RingBuff_Init(RingBuff_t *ringBuff)
{
	//初始化相关信息
	ringBuff->Head = 0;
	ringBuff->Tail = 0;
	ringBuff->Lenght = 0;
}

/*******************************************************************************
** 函数名称: Write_RingBuff
** 功能描述: 往环形缓冲区写入u8类型的数据
** 参数说明: data接收数据
**			 ringBuff接收数据缓冲区
** 返回说明: FLASE:环形缓冲区已满，写入失败;TRUE:写入成功
** 描    述:  往环形缓冲区写入u8类型的数据
** 创建人员: CLTian
** 创建日期: 2019-08-21
**------------------------------------------------------------------------------
********************************************************************************/

uint8_t Write_RingBuff(uint8_t data,RingBuff_t *ringBuff)
{
	if(ringBuff->Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
	{
		return 0;
	}
	ringBuff->Ring_Buff[ringBuff->Tail]=data;
	//ringBuff.Tail++;
	ringBuff->Tail = (ringBuff->Tail+1)%RINGBUFF_LEN;//防止越界非法访问
	ringBuff->Lenght++;
	return 1;
}

/*******************************************************************************
** 函数名称: Read_RingBuff
** 功能描述: 从环形缓冲区读取一个u8类型的数据
** 参数说明: rData:用于保存读取的数据
**			 ringBuff: 数据所在环形缓冲区
** 返回说明: FLASE:环形缓冲区没有数据，读取失败;TRUE:读取成功
** 描    述: 从环形缓冲区读取一个u8类型的数据 
** 创建人员: CLTian
** 创建日期: 2019-08-21
**------------------------------------------------------------------------------
********************************************************************************/

uint8_t Read_RingBuff(uint8_t *rData,RingBuff_t *ringBuff)
{
	if(ringBuff->Lenght == 0)//判断非空
	{
		return 0;
	}
	*rData = ringBuff->Ring_Buff[ringBuff->Head];//先进先出FIFO，从缓冲区头出
	//ringBuff.Head++;
	ringBuff->Head = (ringBuff->Head+1)%RINGBUFF_LEN;//防止越界非法访问
	ringBuff->Lenght--;
	return 1;
}



void RingBuf_Write(unsigned char data,RingBuff_t *ringBuff,uint16_t Length)
{
	ringBuff->Ring_Buff[ringBuff->Tail]=data;//从尾部追加
	if(++ringBuff->Tail>=Length)//尾节点偏移
		ringBuff->Tail=0;//大于数组最大长度 归零 形成环形队列
	if(ringBuff->Tail==ringBuff->Head)//如果尾部节点追到头部节点，则修改头节点偏移位置丢弃早期数据
	{
		if((++ringBuff->Head)>=Length)
			ringBuff->Head=0;
	}
}

uint8_t RingBuf_Read(unsigned char* pData,RingBuff_t *ringBuff)
{
	if(ringBuff->Head==ringBuff->Tail)  return 1;//如果头尾接触表示缓冲区为空
	else
	{
		*pData=ringBuff->Ring_Buff[ringBuff->Head];//如果缓冲区非空则取头节点值并偏移头节点
		if((++ringBuff->Head)>=RINGBUFF_LEN)   ringBuff->Head=0;
		return 0;//返回0，表示读取数据成功
	}
}

