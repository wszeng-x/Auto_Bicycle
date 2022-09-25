#include "detect.h"
#include "stdio.h"

void Init_One_Module(Module_State_t* self, uint16_t cnt_num)
{
	self->cnt = 0;
	self->reload_cnt = cnt_num;
	self->time_out_flag = 1;            //初始化为未上线
	self->last_time_out_flag = 1;
}

void Module_State_Detect(Module_State_t* self)
{
	self->last_time_out_flag = self->time_out_flag;   //更新状态标志
	if(self->cnt > 0)
	{
		self->cnt--;
		self->time_out_flag = 0;   //模块在线
		if(self->last_time_out_flag == 1) //flag 1->0, 模块上线
		{
			//上线to do
		}
	}
	else if(self->cnt == 0)
	{
		self->time_out_flag = 1;  //模块离线
		if(self->last_time_out_flag == 0) //flag 0->1, 模块离线
		{
			//掉线to do
		}
	}
}

void Module_State_Reload(Module_State_t* self)
{
	self->cnt = self->reload_cnt;
}
