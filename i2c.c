#include "i2c.h"
#include "intrins.h"



void delay()   //8us左右的延时函数
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
}		 //起始函数

void stop()
{
	sda=0;
	delay();
	scl=1;
	delay();
	sda=1;
	delay();
}			 //停止函数

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
}			 //	  应答函数，在第9个脉冲时，当scl=1，sda应给与一个低电位作为应答
			 //   如若一直没有应答，则在一段时间后，默认其应答。

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
	sda=1;	 //一个字节发送后，应将sda拉高，scl拉低，为应答做准备。
	delay();
}	//当temp1左移一位时，溢出的那一位将保存在CY中。一个脉冲读一位数。



uchar i2c_read()
{	 
    uchar temp,i;
	scl=0;
	delay();
	sda=1;
	delay();
	for(i=0;i<8;i++)
	{ 	
	    scl=1;	//读取时，scl=1时，数据sda才是稳定的。
	    delay();
		temp=(temp<<1)|sda;//将temp左移一位再和sda相或，即可将sda保存到temp中
		scl=0;
		delay();	
	}
	return temp;
}




