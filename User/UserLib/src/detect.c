#include "detect.h"
#include "stdio.h"

void Init_One_Module(Module_State_t* self, uint16_t cnt_num)
{
	self->cnt = 0;
	self->reload_cnt = cnt_num;
	self->time_out_flag = 1;            //��ʼ��Ϊδ����
	self->last_time_out_flag = 1;
}

void Module_State_Detect(Module_State_t* self)
{
	self->last_time_out_flag = self->time_out_flag;   //����״̬��־
	if(self->cnt > 0)
	{
		self->cnt--;
		self->time_out_flag = 0;   //ģ������
		if(self->last_time_out_flag == 1) //flag 1->0, ģ������
		{
			//����to do
		}
	}
	else if(self->cnt == 0)
	{
		self->time_out_flag = 1;  //ģ������
		if(self->last_time_out_flag == 0) //flag 0->1, ģ������
		{
			//����to do
		}
	}
}

void Module_State_Reload(Module_State_t* self)
{
	self->cnt = self->reload_cnt;
}
