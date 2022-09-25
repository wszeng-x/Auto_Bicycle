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
const uint8_t* rc_rx_buf[2];     //遥控原始数据指针
uint8_t rc_index;

int main(void)
{
  All_Init();
	//初始化遥控数据指针
	rc_rx_buf[0] = Get_Rc_Bufferx(0); //缓冲区0
	rc_rx_buf[1] = Get_Rc_Bufferx(1); //缓冲区1
	
	while(1)
  {
		if(remoter_data_coming == 1)
		{
			remoter_data_coming = 0;
      rc_index = Get_Avail_Rc_Index();
			Parse_Rc_Data(rc_rx_buf[rc_index],&cur_rc);
			if(Rc_Data_Check(&cur_rc))
			{
				//数据出错,则使用上一帧数据
				Rc_Data_Copy(&cur_rc,&last_rc);
				Reset_Rc();
			}
			  Rc_Data_Copy(&last_rc, &cur_rc); //数据正常，保存数据
		}
		else
		{
			//等待超时，清空遥控数据
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
	
	/*A板24v输出上电*/ 
  Power_Ctrl_Config(); 
	
	/*板载IMU相关初始化*/
	TIM6_Init();
	InitSPI5();
	Mpu_Init();
	Init_Quaternion();
	/*调试串口初始化*/
	Usart3_Init();
	/*遥控相关初始化*/
	Usart1_Init();  
	
	/*底盘相关初始化*/
	Init_Can1();
	
	/*云台相关初始化*/
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
 
