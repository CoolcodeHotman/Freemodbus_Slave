/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

extern UCHAR ExceptionBuf[];
extern int Error_ACKNOWLEDGE;
extern int Frame_send_flag;

extern UCHAR    ucMBAddress;  //改动部分

//static eMBException eException;
//static UCHAR    ucMBAddress;
//static peMBFrameSend peMBFrameSendCur;

/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_TEST_MAX     10000     /*!< 测试用接收数据的总容量 */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state.闲置状态 */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state.发送状态 */
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;
extern int Error_ACKNOWLEDGE;

volatile UCHAR  ucRTUBuf[MB_SER_PDU_SIZE_MAX];
volatile UCHAR  ucRTUBuf_Receive[MB_SER_PDU_SIZE_TEST_MAX];

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

volatile eMBSlaveTimerMode eSlaveCurTimerMode;

static volatile USHORT usRcvBufferPos;
static volatile USHORT usTestRcvBufferPos;

/* ----------------------- Global variables ---------------------------------*/
USHORT usTimerT15_50us;
USHORT usTimerT20_50us;
USHORT usTimerT35_50us;
USHORT Error_State_T15;
USHORT CRC_Error;
USHORT Send_Flag=0;      //发送正常响应帧标志位，该位置1后从机才会发送正常响应帧


extern int test15;
extern int test20;
extern int test35;

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBRTUInit( UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    
    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             * 波特率大于19200使用定值：usTimerT35_50us = 35
             * ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );    T35计算公式
             * ( 3UL * 220000UL ) / ( 2UL * ulBaudRate );    T15计算公式
             * 这usTimerT35_50us 一个单位为50uS，将这个计算结果写到定时器。
             * 每中断一次为50us * usTimerT35_50us 微秒；
             */
             usTimerT15_50us = ( 3UL * 220000UL ) / ( 2UL * ulBaudRate );//T15的定时器
             usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );//T35的定时器
             usTimerT20_50us = ( 2UL * 220000UL ) / ulBaudRate ;//T2定时器 
          
        }
        if( xMBPortTimersInit( ( USHORT ) usTimerT15_50us ) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
    }
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void
eMBRTUStart( void )
{
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable( TRUE, FALSE );
    vMBPortTimersT35Enable(  );

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop( void )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    vMBPortTimersDisable(  );
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
//    BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        vMBPortSolveTimersEnable( );//监视定时器启动
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];//MB_SER_PDU_ADDR_OFF=0

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];//MB_SER_PDU_PDU_OFF=1
//        xFrameReceived = TRUE;
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;
    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    Frame_send_flag=0;   //正常帧的发送标志位清零
    
    
    if( eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        if(Error_ACKNOWLEDGE==1)  //此时发生了0X05错误，需要向主机发送一个请求延时的响应
        {
            ExceptionBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
            ExceptionBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );
            Error_ACKNOWLEDGE=0;
        }
        else                      //没有发生错误，直接发送正常的响应帧
        {
            ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
            ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );
        }

        /* Activate the transmitter. */
        eSndState = STATE_TX_XMIT;
        
        /*发送完成中断，若无一下三行代码，无法进入串口发送完成中断*/
        xMBPortSerialPutByte( ( CHAR )*pucSndBufferCur );
        pucSndBufferCur++;
        usSndBufferCount--;       
        vMBPortSerialEnable( FALSE, TRUE );
        Error_ACKNOWLEDGE=0;
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
xMBRTUReceiveFSM( void )
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

   assert( eSndState == STATE_TX_IDLE );//判断是否是空闲状态

    /* Always read the character. */
    ( void )xMBPortSerialGetByte( ( CHAR * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        vMBPortTimersT15Enable(  );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        vMBPortTimersT15Enable(  );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        ucRTUBuf_Receive[usTestRcvBufferPos++]=ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersT15Enable(  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
         {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
            ucRTUBuf_Receive[usTestRcvBufferPos++]=ucByte;
         }
        else
         {
            eRcvState = STATE_RX_ERROR;
         }
        
        if(Error_State_T15==0)
         {
            vMBPortTimersT15Enable(  );
         }
        else
         {
            Error_State_T15=1;
            vMBPortTimersT35Enable(  );
         }
        break;
    }
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( void )
{
    BOOL            xNeedPoll = FALSE;

    assert( eRcvState == STATE_RX_IDLE );

    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte( ( CHAR )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else             //发送完成
        {
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT_COMPLETE );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( TRUE, FALSE );
            eSndState = STATE_TX_IDLE;
            //启动定时器t35用于主动发送时两帧的间隔
            vMBPortT35SendTimersEnable();
            
        }
        break;
    }

    return xNeedPoll;
}

/*每次进中断必须先关闭定时器，
**增加了t15模式用于鉴别一帧内字符连续，
**
*/
BOOL
xMBRTUTimerExpired( void )
{
    BOOL            xNeedPoll = FALSE;
    
    vMBPortTimersDisable(  );
    
    switch ( eSlaveCurTimerMode )
    {
      
        case MB_TMODE_T15:
          
              vMBPortTimersT20Enable();
              if (usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos)==0)
                {
                  CRC_Error=1;  /*!<该变量仅用于测试>*/  
                }
              else
                {
                  Error_State_T15=1;
                }
              test15++;     /*!<该变量用于测试定时器进入几次t15中断>*/
              break;
                       
        case MB_TMODE_T20:
          
              test20++;
              switch ( eRcvState )
              {
                      /* A frame was received and t35 expired. Notify the listener that
                       * a new frame was received. */
                    case STATE_RX_RCV:
                        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );
                        eRcvState = STATE_RX_IDLE;
                        break;

                        /* An error occured while receiving the frame. */
                    case STATE_RX_ERROR:
                        eRcvState = STATE_RX_IDLE;
                        break;
                
                        /* Function called in an illegal state. */
                    default:
                        assert( ( eRcvState == STATE_RX_INIT ) ||
                              ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
                        eRcvState = STATE_RX_IDLE;
                        break;  
              }
              Error_State_T15=0;         /*一帧结束总要清除t15crc出错标志*/
              break;
              
          case MB_TMODE_T35:      
                
              if(Error_State_T15==0)
                 {
                    test35++;
                    /* Timer t35 expired. Startup phase is finished. */
                    xNeedPoll = xMBPortEventPost( EV_READY );
                    eRcvState = STATE_RX_IDLE;
//                   Send_Flag=1;
                 }
              else
                {
                    xMBPortEventPost(EV_FRAME_INCOMPLETE);
                    eRcvState = STATE_RX_IDLE;
                    Error_State_T15=0;   /*一帧结束总要清除t15crc出错标志*/
                }
//              if( Error_ACKNOWLEDGE==1 )
//              {
//                   Send_Flag=1; 
//                   Error_ACKNOWLEDGE=0;
//                }
              break;         
              
           default:         
              break;                   
    }
    return xNeedPoll;
}

/*
**监视定时器中断
*/
int timer4;     /*观察变量以后删除*/
int timer14_35;
void TIMER_14_ExpiredISR( void )
{
    switch ( eSlaveCurTimerMode )
    {
        case  MB_TMODE_T14:
          { 
            TIM_ClearITPendingBit( TIM4, TIM_IT_Update );   
        //  eException=MB_EX_ACKNOWLEDGE;  
            Error_ACKNOWLEDGE=1;
            Send_exception(  MB_EX_ACKNOWLEDGE  );   /*0x05异常码*/
            timer4++;  
          }
            break;
        
        case  MB_TMODE_T14_35:
          {   
            if( Frame_send_flag==1)
              {        
                (void)xMBPortEventPost( EV_FRAME_SENT );
                timer14_35++;
              }   
          }            
            break;
        
        default :
          
            break; 
    }   
}

/*
**设置定时器模式，用于定时器中断中辨别模式
*/
void vMBSlaveSetCurTimerMode( eMBSlaveTimerMode eMBTimerMode )
{
	eSlaveCurTimerMode = eMBTimerMode;
}