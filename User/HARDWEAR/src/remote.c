#include "remote.h"
#include "delay.h"

#define ROCKER_DATA_CHECK(data) ((data > 660) || (data < -660))
#define SWITCH_DATA_CHECK(data) (!((data == 1) || (data == 2) || (data == 3)))

/**
  * @brief          解析遥控数据
  * @author         HLL
  * @param[in]      遥控器数据结构体变量地址
  * @retval         返回空
  */
void Parse_Rc_Data(volatile const uint8_t *sbus_buf, Rc_Ctrl_t* remoter)
{
	remoter->rc.ch0 = (short)((sbus_buf[0]|(sbus_buf[1] << 8))&0x07ff)-1024;
	remoter->rc.ch1 = (short)(((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff)-1024; 
	remoter->rc.ch2 = (short)(((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - 1024;
	remoter->rc.ch3 = (short)(((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - 1024;
	remoter->rc.ch4 = (short)(((sbus_buf[16]) | (sbus_buf[17] << 8)) & 0x07ff) - 1024;
	remoter->rc.s1  = ((sbus_buf[5] >> 4)& 0x000C) >> 2;                           
	remoter->rc.s2  = ((sbus_buf[5] >> 4)& 0x0003);
	remoter->mouse.x   = sbus_buf[6] | (sbus_buf[7] << 8);                  
	remoter->mouse.y   = sbus_buf[8] | (sbus_buf[9] << 8);                 
	remoter->mouse.z   = sbus_buf[10] | (sbus_buf[11] << 8);                 
	remoter->mouse.press_l = sbus_buf[12];                                           
	remoter->mouse.press_r = sbus_buf[13];   
	remoter->key.value = sbus_buf[14] | (sbus_buf[15] << 8);
}

/**
  * @brief          遥控数据检查
  * @author         HLL
  * @param[in]      遥控器数据结构体变量地址
  * @retval         返回空
  */
uint8_t Rc_Data_Check(Rc_Ctrl_t* remote)
{
	if(ROCKER_DATA_CHECK(remote->rc.ch0) || 
		 ROCKER_DATA_CHECK(remote->rc.ch1) ||
		 ROCKER_DATA_CHECK(remote->rc.ch2) ||
		 ROCKER_DATA_CHECK(remote->rc.ch3) ||
		 ROCKER_DATA_CHECK(remote->rc.ch4) ||
		 SWITCH_DATA_CHECK(remote->rc.s1)  ||
		 SWITCH_DATA_CHECK(remote->rc.s2))
	{
		return 1;
	}
  else
	{
		return 0;
	}

}

//rc2 复制给rc1
void Rc_Data_Copy(Rc_Ctrl_t* rc1, Rc_Ctrl_t* rc2)
{
	rc1->key.value = rc2->key.value;
	
	rc1->mouse.press_l = rc2->mouse.press_l;
	rc1->mouse.press_r = rc2->mouse.press_r;
	rc1->mouse.x = rc2->mouse.x;
	rc1->mouse.y = rc2->mouse.y;
	rc1->mouse.z = rc2->mouse.z;
	
	rc1->rc.ch0 = rc2->rc.ch0;
	rc1->rc.ch1 = rc2->rc.ch1;
	rc1->rc.ch2 = rc2->rc.ch2;
	rc1->rc.ch3 = rc2->rc.ch3;
	rc1->rc.ch4 = rc2->rc.ch4;
	rc1->rc.s1 = rc2->rc.s1;
	rc1->rc.s2 = rc2->rc.s2;
}
//清空遥控数据
void Clear_Rc_Data(Rc_Ctrl_t* rc)
{
	rc->key.value = 0;
	
	rc->mouse.press_l = 0;
	rc->mouse.press_r = 0;
	rc->mouse.x = 0;
	rc->mouse.y = 0;
	rc->mouse.z = 0;
	
	rc->rc.ch0 = 0;
	rc->rc.ch1 = 0;
	rc->rc.ch2 = 0;
	rc->rc.ch3 = 0;
	rc->rc.ch4 = 0;
	rc->rc.s1 = 3;
	rc->rc.s2 = 3;
}


uint8_t Get_Avail_Rc_Index(void)
{
	if(DMA_GetCurrentMemoryTarget(DMA2_Stream2))
	{
		return 0;   //DMA使用缓冲区1，CPU可访问缓冲区0
	}
	return 1;
}

