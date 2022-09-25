#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

/*
����LED��������״̬�������𡢷�ת
BSRRH ��ʾBSRR�Ĵ�����16λ��BRy������һ��BRy��1����������͵�ƽ
BSRRL ��ʾBSRR�Ĵ�����16λ��BSy������һ��BSy��1����������ߵ�ƽ
*/
#define	PIN_H(port, pin)		   {port->BSRRL = pin;}	 	//����ߵ�ƽ
#define PIN_L(port, pin)		   {port->BSRRH = pin;}	  //����͵�ƽ
#define PIN_TOGGLE(port, pin)  {port->ODR ^= pin;}   //�����ת״̬
/*��*/
#define LED_GREEN_ON			   PIN_L(GPIOF,GPIO_Pin_14) //����LED_GREEN
#define LED_RED_ON			   	 PIN_L(GPIOE,GPIO_Pin_11)
#define LED_LASER_ON			   PIN_H(GPIOG,GPIO_Pin_13)

/*��*/
#define LED_GREEN_OFF		       PIN_H(GPIOF,GPIO_Pin_14)
#define LED_RED_OFF		   	     PIN_H(GPIOE,GPIO_Pin_11)
#define LED_LASER_OFF			     PIN_L(GPIOG,GPIO_Pin_13)

/*LED ��ת*/									
#define LED_GREEN_TOGGLE		   PIN_TOGGLE(GPIOF,GPIO_Pin_14) 						
#define LED_RED_TOGGLE		   	 PIN_TOGGLE(GPIOE,GPIO_Pin_11) 
#define LED_LASER_TOGGLE		   PIN_TOGGLE(GPIOG,GPIO_Pin_13)

/*LED����*/
#define LED_ARRAY_ON(i)        PIN_L(GPIOG,GPIO_Pin_8 >>i)
#define LED_ARRAY_OFF(i)       PIN_H(GPIOG,GPIO_Pin_8 >>i)
#define LED_ARRAY_TOGGLE(i)    PIN_TOGGLE(GPIOG,GPIO_Pin_8 >>i)


void Init_Led(void);
void Led_Flow_On(void);
void Led_Flow_Off(void);

#endif /*__LED_H__*/

