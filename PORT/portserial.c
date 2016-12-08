/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */
/*
* @brief ���ƽ��պͷ���״̬
* @param xRxEnable ����ʹ�ܡ�
* xTxEnable ����ʹ��
* @retval None
*/
#include "port.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/

/*
**����ʹ�ܺ�����
*/
void 
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
  if(xRxEnable==TRUE)
    {
      /*�򿪴��ڷ��͹��ܺʹ��ڷ����ж�*/ 
     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    }
  else
    {
      /*�رմ��ڷ��͹��ܺʹ��ڷ����ж�*/ 
     USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    }
  if(xTxEnable==TRUE)
    {
      //ʹ�ܷ�������ж�
      USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
  else
    {
      //��ֹ��������ж�
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
}
/**
** @brief ���ڳ�ʼ��
** @param ucPORT ���ں�
** ulBaudRate ������
** ucDataBits ����λ
** eParity У��λ 
** @retval None
**/
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    (void)ucPORT; //���޸Ĵ���
    (void)ucDataBits; //���޸�����λ����
    (void)eParity; //���޸�У���ʽ

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    //ʹ��USART1��GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
    RCC_APB2Periph_USART1, ENABLE);

    //GPIOA.9 USART1_Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //GPIOA.10 USART1_Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = ulBaudRate; //ֻ�޸Ĳ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //���ڳ�ʼ��
    USART_Init(USART1, &USART_InitStructure);
    //ʹ��USART1
    USART_Cmd(USART1, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //�趨USART1 �ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
/*
**  �������485���ͺͽ���ģʽ  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    //GPIOD.8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 
*/
    return TRUE;
}
/*
**���ڷ��ͺ�����
*/
BOOL 
xMBPortSerialPutByte( CHAR ucByte ) 
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    USART_SendData(USART1,ucByte);
    return TRUE;
}
/*
**���ڽ��պ�����
*/
BOOL 
xMBPortSerialGetByte( CHAR * pucByte )  
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    * pucByte = USART_ReceiveData(USART1);
    return TRUE;
}
/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
/*
**���ڷ�������жϺ�����
*/
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
/*
**���ڽ����жϷ�������
*/
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
/*
**STM32��USART1�жϺ���
*/
void USART1_IRQHandler(void)
{
    //���������ж�
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
      prvvUARTRxISR(); 
      //����жϱ�־λ 
      USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
      //��������ж�
    }
  if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
   {
      prvvUARTTxReadyISR();
      //����жϱ�־
      USART_ClearITPendingBit(USART1, USART_IT_TXE);
   }     
}