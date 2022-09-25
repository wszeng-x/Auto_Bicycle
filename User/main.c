#include "stm32f4xx.h"

#include "led.h"
#include "power.h"

#include "usart1.h"
#include "usart3.h"
#include "spi5.h"
#include "time6.h"
#include "imu6500.h"

#include "can1.h"
#include "can2.h"
#include "remote.h"
static void All_Init(void);
static void Print_Logo(void);

uint8_t remoter_data_coming = 0;
Rc_Ctrl_t cur_rc;
Rc_Ctrl_t last_rc;
const uint8_t* rc_rx_buf[2];     //ң��ԭʼ����ָ��
uint8_t rc_index;

int main(void)
{
  All_Init();
	//��ʼ��ң������ָ��
	rc_rx_buf[0] = Get_Rc_Bufferx(0); //������0
	rc_rx_buf[1] = Get_Rc_Bufferx(1); //������1
	
	while(1)
  {
		if(remoter_data_coming == 1)
		{
			remoter_data_coming = 0;
      rc_index = Get_Avail_Rc_Index();
			Parse_Rc_Data(rc_rx_buf[rc_index],&cur_rc);
			if(Rc_Data_Check(&cur_rc))
			{
				//���ݳ���,��ʹ����һ֡����
				Rc_Data_Copy(&cur_rc,&last_rc);
				Reset_Rc();
			}
			  Rc_Data_Copy(&last_rc, &cur_rc); //������������������
		}
		else
		{
			//�ȴ���ʱ�����ң������
			Clear_Rc_Data(&last_rc);
			Clear_Rc_Data(&cur_rc);
		}
      delay_ms(500);
	}	
}



static void All_Init(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	Init_Led();
	
	/*A��24v����ϵ�*/ 
  Power_Ctrl_Config(); 
	
	/*����IMU��س�ʼ��*/
	TIM6_Init();
	InitSPI5();
	Mpu_Init();
	Init_Quaternion();
	/*���Դ��ڳ�ʼ��*/
	Usart3_Init();
	/*ң����س�ʼ��*/
	Usart1_Init();  
	
	/*������س�ʼ��*/
	Init_Can1();
	
	/*��̨��س�ʼ��*/
	Init_Can2();
	
	Print_Logo();
}


static void Print_Logo(void)
{
	USART3_PRINT("\r\n");
	USART3_PRINT("        *===*   *===*    *===*          *===*\r\n");
	USART3_PRINT("       /   /   /   /    /   /          /   /\r\n");
	USART3_PRINT("      /   *===*   /    /   /          /   /\r\n");
	USART3_PRINT("     /           /    /   /          /   /\r\n");
	USART3_PRINT("    /   *===*   /    /   *======*   /   *======*\r\n");
	USART3_PRINT("   /   /   /   /    /          /   /          /\r\n");
	USART3_PRINT("  *===*   *===*    *==========*   *==========*\r\n");
	USART3_PRINT("\r\n");
}
 
