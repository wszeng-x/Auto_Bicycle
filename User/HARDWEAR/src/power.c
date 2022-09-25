#include "power.h"

/**
	* @brief          A��24V��Դ�����������
  * @author         HLL
  * @param[in]      ��
  * @retval         ���ؿ�
  */
void Power_Ctrl_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       
    GPIO_Init(GPIOH, &GPIO_InitStructure);
	
		//24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        Power_Ctrl_Off(i);
			  delay_us(100);
				Power_Ctrl_On(i);
    }
}

/**
	* @brief          ��A��24V��Դ���
  * @author         HLL
  * @param[in]      1,2,3,4 24V��Դ�ڱ��
  * @retval         ���ؿ�
  */
void Power_Ctrl_On(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_SetBits(GPIOH, GPIO_Pin_2 << num);
}

/**
	* @brief          �ر�A��24V��Դ���
  * @author         HLL
  * @param[in]      1,2,3,4 24V��Դ�ڱ��
  * @retval         ���ؿ�
  */
void Power_Ctrl_Off(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_ResetBits(GPIOH, GPIO_Pin_2 << num);
}
