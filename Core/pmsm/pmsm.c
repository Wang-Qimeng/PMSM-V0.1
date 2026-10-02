#include "pmsm.h"

#define CURRENT_CALIB_NUM  256   //零点标定采样次数  256*100us=26ms

float I_uvw[3] = {0.0f, 0.0f, 0.0f};//电机三相电流
float I_offset[3] = {0.0f, 0.0f, 0.0f};//三相电流零点(ADC原始码)
float power; //母线电压
float temper;//母线温度
uint16_t ADC_DMABuff[2];//母线电压，母线电流采集
static void Motor_GetTemperature(void);

void Motor_Init(void)
{
	/*PWM通道打开*/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /*ADC配置*/
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_ADCEx_InjectedStart(&hadc3);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_DMABuff,2);
	
  //初始化电流环PID
  PID_Init(&Current_D_PID);
  PID_Init(&Current_Q_PID);		
  FOC_Init();
}




/*
@function:定时器通道4触发ADC三重注入通道采样，ADC1+DMA规则通道测量电压和温度
@brief:无感电机运行函数
*/
void Motor_Toggle(void)
{
	if(rtX.Dir==Clw)
	{
		rtX.Dir=Cow;
	}
	else
	{
		rtX.Dir=Clw;
	}
}


void Motor_Run(void)
{
    
	
	/*零点标定累加器*/
	static uint32_t calib_sum[3] = {0, 0, 0};
	static uint16_t calib_cnt = 0;

	/*软件模拟，theta角度增加*/
	static float theta=0.0f;
	if(rtX.Dir==Clw)
	theta+=0.01f;
	else if(rtX.Dir==Cow)
	theta-=0.01f;
	
	if(theta>2*PI)
	{
		theta-=2*PI;
	}
	else if(theta<0)
	{
		theta+=2*PI;
	}
	rtX.theta=theta;

	
	/*标定期间强制零矢量：
	  Vd=Vq=0 -> Valpha=Vbeta=0 -> sector=0 -> Tcmp保持初值0 -> CCR=0
	  三相下桥臂全开、上桥臂全关 = 零矢量000，相电流为0*/
	if(calib_cnt < CURRENT_CALIB_NUM)
	{
		Voltage_DQ.Vq = 0.0f;
	}

    /*注入通道对三相电流采样和转换*/
	if(calib_cnt < CURRENT_CALIB_NUM)
	{
		/*零电流状态下累加零点，注意通道对应关系 U->ADC2 V->ADC1 W->ADC3*/
		calib_sum[0] += ADC2->JDR1;
		calib_sum[1] += ADC1->JDR1;
		calib_sum[2] += ADC3->JDR1;
		calib_cnt++;
		if(calib_cnt == CURRENT_CALIB_NUM)
		{
			I_offset[0] = (float)calib_sum[0] / CURRENT_CALIB_NUM;
			I_offset[1] = (float)calib_sum[1] / CURRENT_CALIB_NUM;
			I_offset[2] = (float)calib_sum[2] / CURRENT_CALIB_NUM;
			Voltage_DQ.Vq = 1.5f;    /*标定完成，恢复输出电压*/
		}
	}

    I_uvw[0] = (ADC2->JDR1 - I_offset[0]) * 3.3f / 4096.0f / 0.12f;//u相
    I_uvw[1] = (ADC1->JDR1 - I_offset[1]) * 3.3f / 4096.0f / 0.12f;//v相
    I_uvw[2] = (ADC3->JDR1 - I_offset[2]) * 3.3f / 4096.0f / 0.12f;//w相
	FOC_Run();
	
	/*设置PWM占空比*/
	TIM1->CCR1=rtY.Tcmp1;
	TIM1->CCR2=rtY.Tcmp2;
	TIM1->CCR3=rtY.Tcmp3;

	/*电压,温度采样*/
	power=ADC_DMABuff[1]*3.3f/4096.0f*25.0f;
	Motor_GetTemperature();
		
}

void Motor_GetTemperature(void)
{
	float Rt,temp;
	/*温度采集*/
	Rt=3.3f*4700.0f/(ADC_DMABuff[0]*3.3f/4096.0f)-4700.0f;
	temp=Rt/10000.0f;
	temp=log(temp);
	temp/=3380.0f;
	temp+=(1.0f/(273.15f+25.0f));
	temp=1.0f/(temp);
	temp-=273.15f;
	temper=temp;
	 
}
