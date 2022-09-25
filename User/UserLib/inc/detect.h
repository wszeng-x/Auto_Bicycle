#ifndef __DETECT_H
#define __DETECT_H

#include "stm32f4xx.h"

typedef struct
{
	uint16_t cnt;
	uint16_t reload_cnt;
	
	uint8_t  time_out_flag;
	uint8_t  last_time_out_flag;
 
} Module_State_t;

void Init_One_Module(Module_State_t* self, uint16_t cnt_num);
void Module_State_Detect(Module_State_t* self);
void Module_State_Reload(Module_State_t* self);

#endif /*__DETECT_H*/
