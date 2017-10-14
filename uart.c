#include "uart.h"
#include<string.h>

#define Uart1_Buf_Max 20               //�������ݻ��泤��
#define	BaudRate2		9600UL	//ѡ������
#define	Timer2_Reload	(65536UL -(MAIN_Fosc / 4 / BaudRate2))		//Timer 2 ��װֵ�� ��Ӧ300KHZ

xdata u8 Rec_Buf[Uart1_Buf_Max];  //�������ݻ���
xdata u8 point1 = 0;             //練�ָ��

#define Uart2_Buf_Max 20               //�������ݻ��泤��
xdata u8 Uart2_Rec_Buf[Uart2_Buf_Max];  //�������ݻ���
xdata u8 point2 = 0;             //練�ָ��
bit	B_TX2_Busy = 0;	//����æ��־


void UartInit(void)		//9600bps@11.0592MHz
{
//ע��: STC15W4K32S4ϵ�е�оƬ,�ϵ��������PWM��ص�IO�ھ�Ϊ
//      ����̬,�轫��Щ������Ϊ׼˫��ڻ�ǿ����ģʽ��������ʹ��
//���IO: P0.6/P0.7/P1.6/P1.7/P2.1/P2.2
//        P2.3/P2.7/P3.7/P4.2/P4.4/P4.5


	S2CON &= ~(1<<7);	//8λ����
	P_SW2 &= ~1;		//UART2 ʹ��P1.0 P1.1��	Ĭ��
//	P_SW2 |=  1;		//UART2 ʹ��P4.6 P4.7��

	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	T2H = (u8)(Timer2_Reload >> 8);
	T2L = (u8)Timer2_Reload;
	IE2   |=  1;		//�����ж�
	S2CON |=  (1<<4);	//�������
	AUXR |=  (1<<4);	//Timer run enable

	UART2_INT_ENABLE();
	ES = 1;
	EA = 1;
}

/*----------------------------
���ʹ�������
----------------------------*/
void SendData(unsigned char ch)
{
    SBUF = ch;                 //д���ݵ�UART���ݼĴ���
		while(TI == 0);
		TI = 0;
}

/*----------------------------
�����ַ���
----------------------------*/
void SendString(char *s)
{
    while (*s)                  //����ַ���������־
    {
        SendData(*s++);         //���͵�ǰ�ַ�
    }
}

bit Hand(unsigned char *a)                   // ��������ʶ����
{ 
    if(strstr(Rec_Buf,a)!=NULL)
	    return 1;
	else
		return 0;
}

void CLR_Buf(void)                           // ���ڻ�������
{
	memset(Rec_Buf, 0, Uart1_Buf_Max);      //���

    point1 = 0;                    
}

void Usart() interrupt 4 using 1            // �����жϺ���
{
	ES = 0;
	if (RI)
    {
      RI = 0;                                //���RIλ
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
        TI = 0;                 //���TIλ

    }
		ES =  1;
}

/*----------------------------
ͨ������2���ʹ�������
----------------------------*/
void Uart2SendData(unsigned char ch)
{
	while(B_TX2_Busy);	//��æ��������
    S2BUF = ch;                 //д���ݵ�UART���ݼĴ���
	B_TX2_Busy = 1;
}

/*----------------------------
ͨ������2�����ַ���
----------------------------*/
void Uart2SendString(char *s)
{
    while (*s)                  //����ַ���������־
    {
        Uart2SendData(*s++);         //���͵�ǰ�ַ�
    }
}

bit Uart2Hand(unsigned char *a)                   // ��������ʶ����
{ 
    if(strstr(Uart2_Rec_Buf,a)!=NULL)
	    return 1;
	else
		return 0;
}

void Uart2CLR_Buf(void)                           // ���ڻ�������
{
	memset(Uart2_Rec_Buf, 0, Uart2_Buf_Max);      //���

    point2 = 0;                    
}

void Usart2() interrupt 8 using 1 
{
	UART2_INT_DISABLE();	
	if(RI2)
    {
		CLR_RI2();                               //���RIλ
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
        CLR_TI2();                //���TIλ
		B_TX2_Busy = 0;	//���æ��־

    }
	UART2_INT_ENABLE();	
}

