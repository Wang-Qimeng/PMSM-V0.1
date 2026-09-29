#ifndef __PMSM_H
#define __PMSM_H
#include "main.h"
#include "math.h"
extern float I_uvw[3];
extern float power;
extern float temper;
extern uint16_t ADC_DMABuff[2];
void Motor_Init(void);
void Motor_Run(void);
void Motor_GetTemperature(void);
void Motor_Toggle(void);
#endif

