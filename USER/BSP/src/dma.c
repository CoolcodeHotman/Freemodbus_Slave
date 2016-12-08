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
#include "stm32f10x_dma.h"
extern volatile UCHAR  ucRTUBuf_Receive[];

int posDMA_Buff;

void DMA_Config()
{
    //---------------------串口功能配置---------------------  
    //打开串口对应的外设时钟    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);     
    //启动DMA时钟  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
    //DMA发送中断设置  
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
    //DMA1通道4配置  
    DMA_DeInit(DMA1_Channel7);  
    //外设地址  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);  
    //内存地址  
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&ucRTUBuf_Receive[posDMA_Buff]);  
    //dma传输方向单向  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    //设置DMA在传输时缓冲区的长度  
    DMA_InitStructure.DMA_BufferSize = 8;  
    //设置DMA的外设递增模式，一个外设  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    //设置DMA的内存递增模式  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    //外设数据字长  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    //内存数据字长  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
    //设置DMA的传输模式  
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //设置DMA的优先级别  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
    //设置DMA的2个memory中的变量互相访问  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel7,&DMA_InitStructure);
    
    //配置DMA发送完成后产生中断
    DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);  
    
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);    
    //使能通道4  
//  DMA_Cmd(DMA1_Channel7, ENABLE); 


}



void DMA1_Channel7_IRQHandler(void)  
{  
  DMA_InitTypeDef  DMA_InitStructure;
  if(DMA_GetFlagStatus(DMA1_FLAG_TC7)==SET)
  {
    //清除标志位  
    DMA_ClearFlag(DMA1_FLAG_TC7);  
    posDMA_Buff=posDMA_Buff+8;
  }
    DMA_Cmd(DMA1_Channel7,DISABLE);
    DMA_Config();  
//    DMA1_Channel5->CMAR =  (u32)(&ucRTUBuf_Receive[posDMA_Buff]);;
//    DMA1_Channel5->CNDTR = 8;
//    DMA_SetCurrDataCounter(DMA1_Channel5,8);
//  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&ucRTUBuf_Receive[posDMA_Buff]); 
//  DMA_InitStructure.DMA_BufferSize = 8;  
}


void DMA_Send()
{
//  DMA_InitTypeDef  DMA_InitStructure;
  DMA_Cmd(DMA1_Channel7, ENABLE); 
//  DMA1_Channel5->CMAR =  (u32)(&ucRTUBuf_Receive[posDMA_Buff]);;
//  DMA1_Channel5->CNDTR = 8;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&ucRTUBuf_Receive[posDMA_Buff]);
//  DMA_InitStructure.DMA_BufferSize = 8; 
}
