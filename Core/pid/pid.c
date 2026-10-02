#include "pid.h"

 pid_t Current_D_PID={
    .kp = 1.0f,
    .ki = 0.001f,
    .kd=0.0f,   
    .up_limit = 5.0f,
    .down_limit = -5.0f,
    };
    pid_t Current_Q_PID={
        .kp = 1.0f,
        .ki = 0.001f,
        .kd=0.0f,
        .up_limit = 5.0f,
        .down_limit = -5.0f,
       
    };


void PID_Init(pid_t *pid)
{
    pid->err0 = 0;
	pid->err1 = 0;
	pid->err_int = 0;  //4
}


void PID_Calc(pid_t *pid,float ref,float input,float* output)
{
  
	pid->err1 = pid->err0;
	pid->err0 = ref - input;
     if (pid->ki != 0)		
	{
		pid->err_int += pid->err0 ;
       if(pid->err_int >pid->up_limit/pid->ki)
       {
           pid->err_int=pid->up_limit/pid->ki;
       }
       else if(pid->err_int <pid->down_limit/pid->ki)
       {
           pid->err_int=pid->down_limit/pid->ki;
       }
		
	}
	else
	{
		pid->err_int = 0;
	}
	
	*output = pid->kp * pid->err0
		   + pid->ki * pid->err_int
		   + pid->kd * (pid->err0 - pid->err1) ;
		
	
		
	if (*output > pid->up_limit) {*output = pid->up_limit;}
    else if (*output < pid->down_limit) {*output = pid->down_limit;}
    

   
    

}





