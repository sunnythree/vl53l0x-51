#include "uart.h"
#include<string.h>

#define Uart1_Buf_Max 20               //串口数据缓存长度
#define	BaudRate2		9600UL	//选择波特率
#define	Timer2_Reload	(65536UL -(MAIN_Fosc / 4 / BaudRate2))		//Timer 2 重装值， 对应300KHZ

xdata u8 Rec_Buf[Uart1_Buf_Max];  //串口数据缓存
xdata u8 point1 = 0;             //绶存指针

#define Uart2_Buf_Max 20               //串口数据缓存长度
xdata u8 Uart2_Rec_Buf[Uart2_Buf_Max];  //串口数据缓存
xdata u8 point2 = 0;             //绶存指针
bit	B_TX2_Busy = 0;	//发送忙标志


void UartInit(void)		//9600bps@11.0592MHz
{
//注意: STC15W4K32S4系列的芯片,上电后所有与PWM相关的IO口均为
//      高阻态,需将这些口设置为准双向口或强推挽模式方可正常使用
//相关IO: P0.6/P0.7/P1.6/P1.7/P2.1/P2.2
//        P2.3/P2.7/P3.7/P4.2/P4.4/P4.5


	S2CON &= ~(1<<7);	//8位数据
	P_SW2 &= ~1;		//UART2 使用P1.0 P1.1口	默认
//	P_SW2 |=  1;		//UART2 使用P4.6 P4.7口

	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	T2H = (u8)(Timer2_Reload >> 8);
	T2L = (u8)Timer2_Reload;
	IE2   |=  1;		//允许中断
	S2CON |=  (1<<4);	//允许接收
	AUXR |=  (1<<4);	//Timer run enable

	UART2_INT_ENABLE();
	ES = 1;
	EA = 1;
}

/*----------------------------
发送串口数据
----------------------------*/
void SendData(unsigned char ch)
{
    SBUF = ch;                 //写数据到UART数据寄存器
		while(TI == 0);
		TI = 0;
}

/*----------------------------
发送字符串
----------------------------*/
void SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        SendData(*s++);         //发送当前字符
    }
}

bit Hand(unsigned char *a)                   // 串口命令识别函数
{ 
    if(strstr(Rec_Buf,a)!=NULL)
	    return 1;
	else
		return 0;
}

void CLR_Buf(void)                           // 串口缓存清理
{
	memset(Rec_Buf, 0, Uart1_Buf_Max);      //清空

    point1 = 0;                    
}

void Usart() interrupt 4 using 1            // 串口中断函数
{
	ES = 0;
	if (RI)
    {
      RI = 0;                                //清除RI位
		Rec_Buf[point1] = SBUF; 
//		if (Rec_Buf[0] == 0xd9)
//			{
//				IAP_CONTR = 0x60;
//			}
			
		point1++;               
		if(point1>=Uart1_Buf_Max)          
		{
			point1 = 0;
		}           

    }
    if (TI)
    {
        TI = 0;                 //清除TI位

    }
		ES =  1;
}

/*----------------------------
通过串口2发送串口数据
----------------------------*/
void Uart2SendData(unsigned char ch)
{
	while(B_TX2_Busy);	//不忙后发送数据
    S2BUF = ch;                 //写数据到UART数据寄存器
	B_TX2_Busy = 1;
}

/*----------------------------
通过串口2发送字符串
----------------------------*/
void Uart2SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        Uart2SendData(*s++);         //发送当前字符
    }
}

bit Uart2Hand(unsigned char *a)                   // 串口命令识别函数
{ 
    if(strstr(Uart2_Rec_Buf,a)!=NULL)
	    return 1;
	else
		return 0;
}

void Uart2CLR_Buf(void)                           // 串口缓存清理
{
	memset(Uart2_Rec_Buf, 0, Uart2_Buf_Max);      //清空

    point2 = 0;                    
}

void Usart2() interrupt 8 using 1 
{
	UART2_INT_DISABLE();	
	if(RI2)
    {
		CLR_RI2();                               //清除RI位
		Uart2_Rec_Buf[point2] = S2BUF; 
//		if (Rec_Buf[0] == 0xd9)
//			{
//				IAP_CONTR = 0x60;
//			}
			
		point2++;               
		if(point2>=Uart2_Buf_Max)          
		{
			point2 = 0;
		}           

    }
    if (TI2)
    {
        CLR_TI2();                //清除TI位
		B_TX2_Busy = 0;	//清空忙标志

    }
	UART2_INT_ENABLE();	
}

