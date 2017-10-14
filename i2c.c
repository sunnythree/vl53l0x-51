#include "i2c.h"
#include "intrins.h"



void delay()   //8us���ҵ���ʱ����
{
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
}


void start()
{
	sda=1;
	delay();
	scl=1;
	delay();
	sda=0;
	delay();
}		 //��ʼ����

void stop()
{
	sda=0;
	delay();
	scl=1;
	delay();
	sda=1;
	delay();
}			 //ֹͣ����

void ack()
{
	uchar i=0;
	scl=1;
	delay();
	while((sda==1)&&(i<200))
	{
		i++;	
	}
	scl=0;  
	delay();
}			 //	  Ӧ�������ڵ�9������ʱ����scl=1��sdaӦ����һ���͵�λ��ΪӦ��
			 //   ����һֱû��Ӧ������һ��ʱ���Ĭ����Ӧ��

void i2c_write(uchar dat)
{
	uchar temp1,i;
	temp1=dat;			
	for(i=0;i<8;i++)
	{ 
		temp1<<=1;
		scl=0;
		delay();
		sda=CY;
		delay();
		scl=1;
		delay();		
	}
	scl=0;	  
	delay();
	sda=1;	 //һ���ֽڷ��ͺ�Ӧ��sda���ߣ�scl���ͣ�ΪӦ����׼����
	delay();
}	//��temp1����һλʱ���������һλ��������CY�С�һ�������һλ����



uchar i2c_read()
{	 
    uchar temp,i;
	scl=0;
	delay();
	sda=1;
	delay();
	for(i=0;i<8;i++)
	{ 	
	    scl=1;	//��ȡʱ��scl=1ʱ������sda�����ȶ��ġ�
	    delay();
		temp=(temp<<1)|sda;//��temp����һλ�ٺ�sda��򣬼��ɽ�sda���浽temp��
		scl=0;
		delay();	
	}
	return temp;
}




