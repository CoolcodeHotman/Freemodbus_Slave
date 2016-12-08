/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  ��usart1.c
 * ����    ����printf�����ض���USART1�������Ϳ�����printf��������Ƭ��������
 *           ��ӡ��PC�ϵĳ����ն˻򴮿ڵ������֡�         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�------------------------
 *          | PA9  - USART1(Tx)      |
 *          | PA10 - USART1(Rx)      |
 *           ------------------------
 * ��汾  ��ST3.5.0
 *
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/

#include "usart1.h"
#include <stdarg.h>
#include "stm32f10x.h"
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"
#include "mbutils.h"
#include "user_mb_app.h"
#include "usart1.h"



extern volatile UCHAR  ucRTUBuf_Receive[];
/*
 * ��������USART1_Config
 * ����  ��USART1 GPIO ����,����ģʽ���á�19200 8-N-1
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void USART2_Config(void)
{
  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART1 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/* USART1 GPIO config */
         /* Configure USART1 Tx (PA.09) as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);    
        /* Configure USART1 Rx (PA.10) as input floating */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
                
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
        USART_Cmd(USART2, ENABLE);
        
            
//        NVIC_InitTypeDef NVIC_InitStructure;
//        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//        //�趨USART1 �ж����ȼ�
//        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&NVIC_InitStructure);
        
}





/*

void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
   {
      USART_ClearITPendingBit(USART2, USART_IT_TXE);
   }     
}

*/
