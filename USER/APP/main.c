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
/*0x03 读保持寄存器的值。保持寄存器主机可以进行读和写*/
//保持寄存器起始地址
#define REG_HOLDING_START 0x0000
//保持寄存器数量
#define REG_HOLDING_NREGS 8

/*0x04 读输入寄存器的值。主机只能进行读不能进行写操作*/
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
**方法一
**将x数组中的元素复制到y数组当中。
*/
void exchange(  )
{
  for( i=0;i<10;i++ )
    y[i]=x[i];
}


/*
**方法二
**将x数组中的元素复制到y数组当中。
**效率比方法一更高。
*/
void exchange(  )
{
  register int *p1,*p2;
  for( p1=x,p2=y;p1<&x[10]; )
    *p2++=*p1++;
}

#endif

