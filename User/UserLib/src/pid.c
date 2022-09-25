#include "pid.h"

float Pid_Calc(Position_Pid_t *pid, float ref, float fdb)
{
  pid->ref = ref;
  pid->fdb = fdb;
	pid->err = pid->ref - pid->fdb;
  pid->last_err = pid->err;
  
  
  pid->p_out  = pid->kp * pid->err;
  pid->i_out += pid->ki * pid->err;
  pid->d_out  = pid->kd * (pid->err - pid->last_err);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
