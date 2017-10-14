#ifndef  __I2C_H
#define  __I2C_H
#include <STC15.H>

#define uchar unsigned char
#define uint unsigned int

sbit scl=P1^2;
sbit sda=P1^3;

void delay();
void start();
void stop();
void ack();
void i2c_write(uchar dat);
uchar i2c_read();

#endif
