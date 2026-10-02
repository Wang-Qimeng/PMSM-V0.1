#ifndef __PID_H__
#define __PID_H__
#include "main.h" 
typedef struct
{


    float kp;
    float ki;
    float kd;
    float err0;         // 当前误差   
    float err1;         // 上一误差
    float err_int;      // 积分误差

    float up_limit;         // 上限
    float down_limit;   // 下限

 
} pid_t;

extern pid_t Current_D_PID;
extern pid_t Current_Q_PID;

/*电流环数学模型
@brief:B站，欧拉电子-无刷电动机控制
*/
#if 0

#define L  0.000595f //线电感
#define R  1.02f
#define wc 1000*PI
#define _kp L*wc
#define _ki R*wc*(0.0001f)



#endif



void PID_Init(pid_t *pid);
void PID_Calc(pid_t *pid,float ref,float input,float* output);


#endif




