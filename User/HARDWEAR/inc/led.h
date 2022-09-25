#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

/*
常用LED控制三种状态：亮、灭、反转
BSRRH 表示BSRR寄存器高16位（BRy），哪一个BRy置1，引脚输出低电平
BSRRL 表示BSRR寄存器低16位（BSy），哪一个BSy置1，引脚输出高电平
*/
#define	PIN_H(port, pin)		   {port->BSRRL = pin;}	 	//输出高电平
#define PIN_L(port, pin)		   {port->BSRRH = pin;}	  //输出低电平
#define PIN_TOGGLE(port, pin)  {port->ODR ^= pin;}   //输出反转状态
/*亮*/
#define LED_GREEN_ON			   PIN_L(GPIOF,GPIO_Pin_14) //板载LED_GREEN
#define LED_RED_ON			   	 PIN_L(GPIOE,GPIO_Pin_11)
#define LED_LASER_ON			   PIN_H(GPIOG,GPIO_Pin_13)

/*灭*/
#define LED_GREEN_OFF		       PIN_H(GPIOF,GPIO_Pin_14)
#define LED_RED_OFF		   	     PIN_H(GPIOE,GPIO_Pin_11)
#define LED_LASER_OFF			     PIN_L(GPIOG,GPIO_Pin_13)

/*LED 反转*/									
#define LED_GREEN_TOGGLE		   PIN_TOGGLE(GPIOF,GPIO_Pin_14) 						
#define LED_RED_TOGGLE		   	 PIN_TOGGLE(GPIOE,GPIO_Pin_11) 
#define LED_LASER_TOGGLE		   PIN_TOGGLE(GPIOG,GPIO_Pin_13)

/*LED阵列*/
#define LED_ARRAY_ON(i)        PIN_L(GPIOG,GPIO_Pin_8 >>i)
#define LED_ARRAY_OFF(i)       PIN_H(GPIOG,GPIO_Pin_8 >>i)
#define LED_ARRAY_TOGGLE(i)    PIN_TOGGLE(GPIOG,GPIO_Pin_8 >>i)


void Init_Led(void);
void Led_Flow_On(void);
void Led_Flow_Off(void);

#endif /*__LED_H__*/

