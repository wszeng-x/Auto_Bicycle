#ifndef  __PID_H
#define  __PID_H

#include "stm32f4xx.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#define NEW_PPID(p, i, d,i_max_limit,out_max_limit)                 \
{                                                                   \
	 .kp = p,                                                         \
	 .ki = i,                                                         \
	 .kd = d,                                                         \
	 .i_max = i_max_limit,                                            \
	 .out_max = out_max_limit,                                        \
}

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value
	float err;
  float last_err;   

  float p_out;
  float i_out;
  float d_out;
  float output;
	
}Position_Pid_t;

float Pid_Calc(Position_Pid_t *pid, float ref, float fdb);

#endif
