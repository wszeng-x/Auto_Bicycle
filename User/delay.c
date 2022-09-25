#include "delay.h"

/*
ժ�ԣ�https://blog.csdn.net/lintax/article/details/83047326
C���Դ��룺
for(; nCount != 0; nCount--);
ִ��һ��ѭ�� ������5��ʱ������ 
stm32 f427��ʼ����ϵͳʱ�Ӵﵽ��ߵ�180Mhz 
������ʱ1us ��Ҫѭ��180/5=36��
*/

void delay_ms(unsigned int ms_count)
{
	unsigned int i=0;
	for(; i<ms_count; i++)
	{
		unsigned short nCount=36000;    
		for(; nCount != 0; nCount--);
	}
}

void delay_us(unsigned int us_count)
{
	unsigned int i=0;
	for(; i<us_count; i++)
	{
		unsigned char nCount=36;
		for(; nCount != 0; nCount--);
	}	
}

void spi_delay(void)
{
	unsigned char nCount=7;         //Լ200ns
	for(; nCount != 0; nCount--);	
}


