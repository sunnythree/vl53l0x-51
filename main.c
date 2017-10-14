#include "main.h"
#include "delay.h"
#include "uart.h"
#include "i2c.h"
#include "vl53l0x.h"
#include "stdio.h"

/*
*����ʹ�õĴ���Ϊ����2����Ӧ����p1^0(rx2)��P1^1(tx2)����
*i2c�����ģ��ģ�ʹ�õ�������p1^2(scl)��p1^3(sdl)
*/
xdata unsigned char buf[30];
xdata unsigned int val = 0;

/******************** ������ **************************/
void main(void)
{
	UartInit();
	Uart2SendString("serial ready\r\n");
	delay_ms(200);
	//���·ֱ��0xc0,0xc1,0xc2�Ĵ�����ֵ��������Ӧ����0xEE,0xAA,0x10
	val = VL53L0X_Read_Byte(IDENTIFICATION_REVISION_ID-2);
	sprintf(buf,"c0=%x\r\n",val);
	Uart2SendString(buf);
	val = VL53L0X_Read_Byte(IDENTIFICATION_REVISION_ID-1);
	sprintf(buf,"c1=%x\r\n",val);
	Uart2SendString(buf);
	val = VL53L0X_Read_Byte(IDENTIFICATION_REVISION_ID);
	sprintf(buf,"val=%x\r\n",val);
	Uart2SendString(buf);

	VL53L0X_init(1);
	startContinuous(0);
		 
	 	
	while(1)
	{
		/*VL53L0X_Write_Byte(SYSRANGE_START, 0x01);
		 cnt = 0;
		 while(cnt <100)
		 {
				delay_ms(10);
				val = VL53L0X_Read_Byte(RESULT_RANGE_STATUS);
				if( val & 0x01) break;
				cnt++;
		 }
	
		 if(!(val & 0x01))Uart2SendString("\r\nnot readey \r\n");
				
		VL53L0X_Read_Len(VL53L0X_Add, 0x14 , 12, gbuf);
		
		count[0] = makeuint16(gbuf[7], gbuf[6]);
		count[1] = makeuint16(gbuf[9], gbuf[8]);
		count[2] = makeuint16(gbuf[11], gbuf[10]);
		DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
		sprintf(buf,"dis =%d\r\n",count[2]);
		Uart2SendString(buf); */
		//������ѵ������������Ϊʹ���ж�����
		val = readRangeContinuousMillimeters();
		sprintf(buf,"val =%d\r\n",val);
		Uart2SendString(buf);
		//delay_ms(1000); 
	}
}




