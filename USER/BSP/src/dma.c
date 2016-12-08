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
    //---------------------���ڹ�������---------------------  
    //�򿪴��ڶ�Ӧ������ʱ��    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);     
    //����DMAʱ��  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
    //DMA�����ж�����  
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
    //DMA1ͨ��4����  
    DMA_DeInit(DMA1_Channel7);  
    //�����ַ  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);  
    //�ڴ��ַ  
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&ucRTUBuf_Receive[posDMA_Buff]);  
    //dma���䷽����  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    //����DMA�ڴ���ʱ�������ĳ���  
    DMA_InitStructure.DMA_BufferSize = 8;  
    //����DMA���������ģʽ��һ������  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    //����DMA���ڴ����ģʽ  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    //���������ֳ�  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    //�ڴ������ֳ�  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
    //����DMA�Ĵ���ģʽ  
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //����DMA�����ȼ���  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
    //����DMA��2��memory�еı����������  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel7,&DMA_InitStructure);
    
    //����DMA������ɺ�����ж�
    DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);  
    
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);    
    //ʹ��ͨ��4  
//  DMA_Cmd(DMA1_Channel7, ENABLE); 


}



void DMA1_Channel7_IRQHandler(void)  
{  
  DMA_InitTypeDef  DMA_InitStructure;
  if(DMA_GetFlagStatus(DMA1_FLAG_TC7)==SET)
  {
    //�����־λ  
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
