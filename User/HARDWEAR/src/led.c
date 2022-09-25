#include "led.h"
#include "delay.h"

void Init_Led(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	//板载LED_GREEN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	//板载LED_RED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//板载LED阵列
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
									   GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | 
									   GPIO_Pin_7 | GPIO_Pin_8;
									
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
		
	//激光
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	//关闭所有灯
	GPIO_SetBits(GPIOG,GPIO_Pin_13);
	GPIO_SetBits(GPIOF,GPIO_Pin_14);
	GPIO_SetBits(GPIOE,GPIO_Pin_11);
	GPIO_SetBits(GPIOG,GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
	                   GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | 
										 GPIO_Pin_7 | GPIO_Pin_8);
	
	Led_Flow_On();
	Led_Flow_Off();
}
void Led_Flow_On(void)
{
	for(int i=0;i<8;i++)
  {
		LED_ARRAY_ON(i);
		delay_ms(100);
	}
}

void Led_Flow_Off(void)
{
	for(int i=0;i<8;i++)
  {
		LED_ARRAY_OFF(i);
		delay_ms(100);
	}
}

