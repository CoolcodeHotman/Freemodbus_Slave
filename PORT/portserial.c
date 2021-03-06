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
* @brief 控制接收和发送状态
* @param xRxEnable 接收使能、
* xTxEnable 发送使能
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
**串口使能函数。
*/
void 
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
  if(xRxEnable==TRUE)
    {
      /*打开串口发送功能和串口发送中断*/ 
     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    }
  else
    {
      /*关闭串口发送功能和串口发送中断*/ 
     USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    }
  if(xTxEnable==TRUE)
    {
      //使能发送完成中断
      USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
  else
    {
      //禁止发送完成中断
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
}
/**
** @brief 串口初始化
** @param ucPORT 串口号
** ulBaudRate 波特率
** ucDataBits 数据位
** eParity 校验位 
** @retval None
**/
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    (void)ucPORT; //不修改串口
    (void)ucDataBits; //不修改数据位长度
    (void)eParity; //不修改校验格式

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    //使能USART1，GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
    RCC_APB2Periph_USART1, ENABLE);

    //GPIOA.9 USART1_Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //GPIOA.10 USART1_Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮动输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = ulBaudRate; //只修改波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //串口初始化
    USART_Init(USART1, &USART_InitStructure);
    //使能USART1
    USART_Cmd(USART1, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //设定USART1 中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
/*
**  最后配置485发送和接收模式  
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
**串口发送函数。
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
**串口接收函数。
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
**串口发送完成中断函数。
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
**串口接收中断服务函数。
*/
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
/*
**STM32的USART1中断函数
*/
void USART1_IRQHandler(void)
{
    //发生接收中断
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
      prvvUARTRxISR(); 
      //清除中断标志位 
      USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
      //发生完成中断
    }
  if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
   {
      prvvUARTTxReadyISR();
      //清除中断标志
      USART_ClearITPendingBit(USART1, USART_IT_TXE);
   }     
}