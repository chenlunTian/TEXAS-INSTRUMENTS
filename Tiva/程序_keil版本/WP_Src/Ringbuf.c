/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, CLTian, China, AH.
**                           All Rights Reserved
**
**                           By(chenlun)
**                           QQ:354769733
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: Ringbuf.c
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
#include "Ringbuf.h"



/*******************************************************************************
** ��������: RingBuff_Init
** ��������: ��ʼ�����λ�����
** ����˵��: ringBuff: [����/��] 
** ����˵��: None
** ��    ��: ��ʼ�����λ����� 
** ������Ա: CLTian
** ��������: 2019-08-21
**------------------------------------------------------------------------------
********************************************************************************/

void RingBuff_Init(RingBuff_t *ringBuff)
{
	//��ʼ�������Ϣ
	ringBuff->Head = 0;
	ringBuff->Tail = 0;
	ringBuff->Lenght = 0;
}

/*******************************************************************************
** ��������: Write_RingBuff
** ��������: �����λ�����д��u8���͵�����
** ����˵��: data��������
**			 ringBuff�������ݻ�����
** ����˵��: FLASE:���λ�����������д��ʧ��;TRUE:д��ɹ�
** ��    ��:  �����λ�����д��u8���͵�����
** ������Ա: CLTian
** ��������: 2019-08-21
**------------------------------------------------------------------------------
********************************************************************************/

uint8_t Write_RingBuff(uint8_t data,RingBuff_t *ringBuff)
{
	if(ringBuff->Lenght >= RINGBUFF_LEN) //�жϻ������Ƿ�����
	{
		return 0;
	}
	ringBuff->Ring_Buff[ringBuff->Tail]=data;
	//ringBuff.Tail++;
	ringBuff->Tail = (ringBuff->Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
	ringBuff->Lenght++;
	return 1;
}

/*******************************************************************************
** ��������: Read_RingBuff
** ��������: �ӻ��λ�������ȡһ��u8���͵�����
** ����˵��: rData:���ڱ����ȡ������
**			 ringBuff: �������ڻ��λ�����
** ����˵��: FLASE:���λ�����û�����ݣ���ȡʧ��;TRUE:��ȡ�ɹ�
** ��    ��: �ӻ��λ�������ȡһ��u8���͵����� 
** ������Ա: CLTian
** ��������: 2019-08-21
**------------------------------------------------------------------------------
********************************************************************************/

uint8_t Read_RingBuff(uint8_t *rData,RingBuff_t *ringBuff)
{
	if(ringBuff->Lenght == 0)//�жϷǿ�
	{
		return 0;
	}
	*rData = ringBuff->Ring_Buff[ringBuff->Head];//�Ƚ��ȳ�FIFO���ӻ�����ͷ��
	//ringBuff.Head++;
	ringBuff->Head = (ringBuff->Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
	ringBuff->Lenght--;
	return 1;
}



void RingBuf_Write(unsigned char data,RingBuff_t *ringBuff,uint16_t Length)
{
	ringBuff->Ring_Buff[ringBuff->Tail]=data;//��β��׷��
	if(++ringBuff->Tail>=Length)//β�ڵ�ƫ��
		ringBuff->Tail=0;//����������󳤶� ���� �γɻ��ζ���
	if(ringBuff->Tail==ringBuff->Head)//���β���ڵ�׷��ͷ���ڵ㣬���޸�ͷ�ڵ�ƫ��λ�ö�����������
	{
		if((++ringBuff->Head)>=Length)
			ringBuff->Head=0;
	}
}

uint8_t RingBuf_Read(unsigned char* pData,RingBuff_t *ringBuff)
{
	if(ringBuff->Head==ringBuff->Tail)  return 1;//���ͷβ�Ӵ���ʾ������Ϊ��
	else
	{
		*pData=ringBuff->Ring_Buff[ringBuff->Head];//����������ǿ���ȡͷ�ڵ�ֵ��ƫ��ͷ�ڵ�
		if((++ringBuff->Head)>=RINGBUFF_LEN)   ringBuff->Head=0;
		return 0;//����0����ʾ��ȡ���ݳɹ�
	}
}

