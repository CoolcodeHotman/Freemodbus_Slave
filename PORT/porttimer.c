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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbframe.h"
#include "core_cm3.h"

/* ----------------------- static functions ---------------------------------*/
static void  prvvTIMERExpiredISR( void );
void TIMER_14_ExpiredISR( void );



/* ----------------------- Variables ----------------------------------------*/
//static eMBException eException;
extern USHORT usTimerT15_50us;
extern USHORT usTimerT20_50us;
extern USHORT usTimerT35_50us;
extern int Frame_send_flag;
extern int Master_RepTime;

int  Error_ACKNOWLEDGE=0;  //x05�������������ӳ���Ӧ��ʱʱ�䣩���ڶ�ʱ��4�б���λ
int  test15 = 0;
int  test20 = 0;
int  test35 = 0;


/* ----------------------- Start implementation -----------------------------*/
/*
**��ʼ���δ�ʱ��
**�ö�ʱ������T15  T2  T35
*/
BOOL 
xMBPortTimersInit( USHORT usTimeOut50us )
{
  SysTick->LOAD  = (usTimeOut50us*3600)-1;
  SysTick->VAL   = 0;
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk; 
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  return TRUE;
}

/*
**��ʼ����ʱ��4��
**�ö�ʱ�����ڼ��Ӵӻ������ִ��ʱ�䡣
**��������������ĺŹ�����ı䡣
*/
BOOL 
xMBPortSolveTimersInit( USHORT MasterRepTime )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  uint16_t PrescalerValue = 0;
  
  //ʹ�ܶ�ʱ��4ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  //��ʱ��ʱ�������˵��
  //HCLKΪ72MHz��APB1����2��ƵΪ36MHz
  //TIM4��ʱ�ӱ�Ƶ��Ϊ72MHz��Ӳ���Զ���Ƶ,�ﵽ���
  //TIM4�ķ�Ƶϵ��Ϊ7199��ʱ���Ƶ��Ϊ72 / (1 + Prescaler) = 10KHz,��׼Ϊ100us
  //TIM������ֵΪMasterRepTime
  PrescalerValue = (uint16_t) (SystemCoreClock / 10000) - 1;
  //��ʱ��1��ʼ��
  TIM_TimeBaseStructure.TIM_Period = (uint16_t) MasterRepTime;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //Ԥװ��ʹ��
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //��ʱ��4�ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������жϱ�־λ
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
  //��ʱ��4����жϹر�
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
  //��ʱ��4����
  TIM_Cmd(TIM4,  DISABLE);
  return TRUE;
}


/*ʹ�ܼ��Ӷ�ʱ��*/
inline void vMBPortSolveTimersEnable(  )
{
   vMBSlaveSetCurTimerMode(MB_TMODE_T14);
   TIM4->ARR=10000;
   TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
   TIM_SetCounter(TIM4,0x0000);
   TIM_Cmd(TIM4,  ENABLE);
   TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}


inline void  vMBPortT35SendTimersEnable(  )
{
    vMBSlaveSetCurTimerMode(MB_TMODE_T14_35);
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    uint16_t PrescalerValue = 0;
    
    //ʹ�ܶ�ʱ��4ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    PrescalerValue = (uint16_t) (SystemCoreClock / 10000) - 1;

    TIM_TimeBaseStructure.TIM_Period = 35;  //3.5ms��ʱ
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    //Ԥװ��ʹ��
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //��ʱ��4�ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    //�������жϱ�־λ
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    //��ʱ��4����жϹر�
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    //��ʱ��4����
    TIM_Cmd(TIM4,  DISABLE);

    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    
    TIM_SetCounter(TIM4,0x0000);
    TIM_Cmd(TIM4,  ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}


inline void  vMBPortT35SendTimersDisable(  )
{
   TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
   TIM_SetCounter(TIM4,0x0000);
   TIM_Cmd(TIM4,  DISABLE);
   TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
}


/*�رռ��Ӷ�ʱ��*/
inline void vMBPortSolveTimersDisable(  )
{
   
   TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
   TIM_SetCounter(TIM4,0x0000);
   TIM_Cmd(TIM4,  DISABLE);
   TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
}


/*********ʹ��T15��ʱ��ʱ��****************/
inline void vMBPortTimersT15Enable(  )
{ 
   vMBSlaveSetCurTimerMode(MB_TMODE_T15);
   SysTick->LOAD  = (usTimerT15_50us*3600)-1;
   SysTick->VAL=0;
   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
}

/*********ʹ��T2��ʱ��ʱ��****************/
inline void vMBPortTimersT20Enable(  )
{ 
   vMBSlaveSetCurTimerMode(MB_TMODE_T20);
   SysTick->LOAD  = (usTimerT20_50us*3600)-1;
   SysTick->VAL=0;
   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


/*********ʹ��T35��ʱ��ʱ��****************/
inline void vMBPortTimersT35Enable(  )
{ 
   vMBSlaveSetCurTimerMode(MB_TMODE_T35);
   SysTick->LOAD  = (usTimerT35_50us*3600)-1;
   SysTick->VAL=0;
   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


/*********�رճ�ʱ��ʱ��***************/
inline void  vMBPortTimersDisable(  )
{
   /* Disable any pending timers. */
   SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}


/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */

static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}



/*
**�δ�ʱ���жϺ���
*/
void SysTick_Handler(void)
{

    prvvTIMERExpiredISR(  );
    
}


void TIM4_IRQHandler(  void  )
{
  if ( TIM_GetITStatus( TIM4, TIM_IT_Update) != RESET )
    {  
      vMBPortSolveTimersDisable(  );
      TIMER_14_ExpiredISR(  );
    }

}
