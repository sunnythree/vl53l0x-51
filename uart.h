#ifndef __UART_H
#define __UART_H

#include	"config.h"

#define		S1_USE_P30P31()		P_SW1 &= ~0xc0					//UART1 使用P30 P31口	默认
#define		S1_USE_P36P37()		P_SW1 = (P_SW1 & ~0xc0) | 0x40	//UART1 使用P36 P37口
#define		S1_USE_P16P17()		P_SW1 = (P_SW1 & ~0xc0) | 0x80	//UART1 使用P16 P17口

#define		TI2					(S2CON & 2) != 0
#define		RI2					(S2CON & 1) != 0
#define		SET_TI2()			S2CON |=  2
#define		CLR_TI2()			S2CON &= ~2
#define		CLR_RI2()			S2CON &= ~1

#define		UART2_INT_ENABLE()		IE2 |=  1	//允许串口2中断
#define		UART2_INT_DISABLE()		IE2 &= ~1	//不允许串口2中断

#define ES2 0x01	//IE2.0


void UartInit();
void SendData(unsigned char ch);
void SendString(char *s);
bit Hand(unsigned char *a) ;
void CLR_Buf(void) ;

void Uart2SendData(unsigned char ch);
void Uart2SendString(char *s);
bit Uart2Hand(unsigned char *a) ;
void Uart2CLR_Buf(void) ;



#endif