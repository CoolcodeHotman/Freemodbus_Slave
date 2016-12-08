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

int  Error_ACKNOWLEDGE=0;  //x05错误（请求主机延长响应超时时间），在定时器4中被置位
int  test15 = 0;
int  test20 = 0;
int  test35 = 0;


/* ----------------------- Start implementation -----------------------------*/
/*
**初始化滴答定时器
**该定时器用于T15  T2  T35
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
**初始化定时器4。
**该定时器用于监视从机命令的执行时间。
**输入参数由主机的号功能码改变。
*/
BOOL 
xMBPortSolveTimersInit( USHORT MasterRepTime )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  uint16_t PrescalerValue = 0;
  
  //使能定时器4时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  //定时器时间基配置说明
  //HCLK为72MHz，APB1经过2分频为36MHz
  //TIM4的时钟倍频后为72MHz（硬件自动倍频,达到最大）
  //TIM4的分频系数为7199，时间基频率为72 / (1 + Prescaler) = 10KHz,基准为100us
  //TIM最大计数值为MasterRepTime
  PrescalerValue = (uint16_t) (SystemCoreClock / 10000) - 1;
  //定时器1初始化
  TIM_TimeBaseStructure.TIM_Period = (uint16_t) MasterRepTime;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //预装载使能
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //定时器4中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //清除溢出中断标志位
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
  //定时器4溢出中断关闭
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
  //定时器4禁能
  TIM_Cmd(TIM4,  DISABLE);
  return TRUE;
}


/*使能监视定时器*/
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
    
    //使能定时器4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    PrescalerValue = (uint16_t) (SystemCoreClock / 10000) - 1;

    TIM_TimeBaseStructure.TIM_Period = 35;  //3.5ms定时
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    //预装载使能
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //定时器4中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    //清除溢出中断标志位
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    //定时器4溢出中断关闭
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    //定时器4禁能
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


/*关闭监视定时器*/
inline void vMBPortSolveTimersDisable(  )
{
   
   TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
   TIM_SetCounter(TIM4,0x0000);
   TIM_Cmd(TIM4,  DISABLE);
   TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
}


/*********使能T15超时定时器****************/
inline void vMBPortTimersT15Enable(  )
{ 
   vMBSlaveSetCurTimerMode(MB_TMODE_T15);
   SysTick->LOAD  = (usTimerT15_50us*3600)-1;
   SysTick->VAL=0;
   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
}

/*********使能T2超时定时器****************/
inline void vMBPortTimersT20Enable(  )
{ 
   vMBSlaveSetCurTimerMode(MB_TMODE_T20);
   SysTick->LOAD  = (usTimerT20_50us*3600)-1;
   SysTick->VAL=0;
   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


/*********使能T35超时定时器****************/
inline void vMBPortTimersT35Enable(  )
{ 
   vMBSlaveSetCurTimerMode(MB_TMODE_T35);
   SysTick->LOAD  = (usTimerT35_50us*3600)-1;
   SysTick->VAL=0;
   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


/*********关闭超时定时器***************/
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
**滴答定时器中断函数
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
