#include "delay.h"

/*
摘自：https://blog.csdn.net/lintax/article/details/83047326
C语言代码：
for(; nCount != 0; nCount--);
执行一次循环 经历了5个时钟周期 
stm32 f427初始化后系统时钟达到最高的180Mhz 
所以延时1us 需要循环180/5=36次
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
	unsigned char nCount=7;         //约200ns
	for(; nCount != 0; nCount--);	
}


