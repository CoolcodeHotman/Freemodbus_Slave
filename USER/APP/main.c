#include "stm32f10x.h"
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"
#include "mbutils.h"
#include "user_mb_app.h"
#include "usart1.h"
/*
int x[10]={0,1,2,3,4,5,6,7,8,9};
int y[10]={2,2,2,2,2,2,2,2,2,2};
void exchange();
*/
/*0x03 �����ּĴ�����ֵ�����ּĴ����������Խ��ж���д*/
//���ּĴ�����ʼ��ַ
#define REG_HOLDING_START 0x0000
//���ּĴ�������
#define REG_HOLDING_NREGS 8

/*0x04 ������Ĵ�����ֵ������ֻ�ܽ��ж����ܽ���д����*/
#define REG_INPUT_START 0

#define REG_INPUT_NREGS 4



void main(  )
{
    USART2_Config();
    DMA_Config();
    eMBInit( MB_RTU, 0x01, 0x01, 9600, MB_PAR_NONE );
    xMBPortSolveTimersInit(  1000  );
    eMBEnable(  );                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    while ( 1 )
      {
          eMBPoll(  );
      }
}

#if 0
/*
**����һ
**��x�����е�Ԫ�ظ��Ƶ�y���鵱�С�
*/
void exchange(  )
{
  for( i=0;i<10;i++ )
    y[i]=x[i];
}


/*
**������
**��x�����е�Ԫ�ظ��Ƶ�y���鵱�С�
**Ч�ʱȷ���һ���ߡ�
*/
void exchange(  )
{
  register int *p1,*p2;
  for( p1=x,p2=y;p1<&x[10]; )
    *p2++=*p1++;
}

#endif

