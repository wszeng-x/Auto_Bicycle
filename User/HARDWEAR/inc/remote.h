#ifndef __REMOTER_H__
#define __REMOTER_H__

#include "stm32f4xx.h"
#include "usart1.h"

#define RC_CHANNAL_ERROR_VALUE 700

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)


/*Ò£¿Ø¿ØÖÆ²¿·Ö*/
#define S1_VALUE  cur_rc.rc.s1
#define S2_VALUE  cur_rc.rc.s2

#define LAST_S1_VALUE  last_rc.rc.s1
#define LAST_S2_VALUE  last_rc.rc.s2

#define S1_CHANGE(a,b) ((LAST_S1_VALUE == (a)) && (S1_VALUE == (b)))
#define S2_CHANGE(a,b) ((LAST_S2_VALUE == (a)) && (S2_VALUE == (b)))



typedef struct Remote
{
	struct
	{ 
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int16_t ch4;
		int16_t s1;
		int16_t s2;
	}rc;
	
	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	
	struct
	{
		uint16_t value;
	}key;
}Rc_Ctrl_t;


void Parse_Rc_Data(volatile const uint8_t *sbus_buf, Rc_Ctrl_t* remoter);
uint8_t Rc_Data_Check(Rc_Ctrl_t* remote);
void Rc_Data_Copy(Rc_Ctrl_t* rc1, Rc_Ctrl_t* rc2);
void Clear_Rc_Data(Rc_Ctrl_t* rc);
uint8_t Get_Avail_Rc_Index(void);
#endif
