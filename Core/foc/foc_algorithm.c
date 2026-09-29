#include "foc_algorithm.h"

/*********************************************************************************************************
*
*                                  定义变量
*
*********************************************************************************************************/
ExtX rtX;            /* 输入结构体变量 */
ExtY rtY;            /* 输出结构体变量 */
Transf_Cos_Sin_DEF Transf_Cos_Sin;         /* 当前角度计算三角函数 */
Voltage_DQ_DEF Voltage_DQ;                 /* DQ电压输出 */
Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta; /* RevPark变换输出 */

Current_ABC_DEF Current_Iabc;                 /*采样出的三相电流*/
Current_Alpha_Beta_DEF Current_Ialpha_beta;   /*中间量：Clarke变换输出*/
Current_Id_Iq_DEF Current_Id_Iq;              /*中间量：Park变换输出*/




/*********************************************************************************************************
*
*                                  函数声明
*
*********************************************************************************************************/
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin);
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta);
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm);
void Clarke_Transf(Current_ABC_DEF current_abc, Current_Alpha_Beta_DEF* current_alpha_beta);
void Park_Transf(Current_Alpha_Beta_DEF current_alpha_beta, Transf_Cos_Sin_DEF cos_sin, Current_Id_Iq_DEF* current_id_iq);
/*********************************************************************************************************
* 功  能: FOC算法初始化
* 形  参: 无
* 说  明: 初始化PI控制器参数
*********************************************************************************************************/
void FOC_Init(void)
{   
    rtX.Udc = 24.0f;    /* 母线电压 */
    rtX.Tpwm = 8400.0f*2;
    rtX.Dir=Clw;    
	Voltage_DQ.Vd=0.0f;
	Voltage_DQ.Vq=1.5f;

    

}

/*

@brief 基于等幅值FOC算法

*/


/*********************************************************************************************************
* 功  能: FOC算法     
* 形  参: 无
* 说  明: 执行FOC控制流程，用PI控制器控制电流环
*********************************************************************************************************/
void FOC_Run(void)
{
    Current_Iabc.Ia=I_uvw[0];
    Current_Iabc.Ib=I_uvw[1];
    Current_Iabc.Ic=I_uvw[2];

   
    Clarke_Transf(Current_Iabc, &Current_Ialpha_beta);          /* Clarke变换 */
    Angle_To_Cos_Sin(rtX.theta, &Transf_Cos_Sin);            /* 计算三角函数 */
    Park_Transf(Current_Ialpha_beta, Transf_Cos_Sin, &Current_Id_Iq); /* Park变换 */


    Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta);    /* Revpark变换 */
    SVPWM_Calc(Voltage_Alpha_Beta, rtX.Udc, rtX.Tpwm);        /* SVPWM */
}

/*********************************************************************************************************
* 功  能: COS_SIN值计算
* 形  参: 角度以及COS_SIN结构体
* 说  明: COS_SIN值计算
*********************************************************************************************************/
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin)
{
    cos_sin->Cos = cosf(angle);
    cos_sin->Sin = sinf(angle);
}

/*********************************************************************************************************
* 功  能: 反PARK变换
* 形  参: DQ轴电压、COS_SIN值、alpha_beta电压
*         Valpha = Vd * cos(theta) - Vq * sin(theta);
*         Vbeta  = Vd * sin(theta) + Vq * cos(theta);
* 说  明: 直流变交流
*********************************************************************************************************/
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta)
{
    v_alpha_beta->Valpha = cos_sin.Cos * v_dq.Vd - cos_sin.Sin * v_dq.Vq;
    v_alpha_beta->Vbeta  = cos_sin.Sin * v_dq.Vd + cos_sin.Cos * v_dq.Vq;
}

/*********************************************************************************************************
* 功  能: SVPWM计算
* 形  参: alpha_beta电压以及母线电压、定时器周期
* 说  明: 根据alpha_beta电压计算三相占空比
*********************************************************************************************************/
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm)
{
    int sector;
    float Tx,Ty,T,Ta,Tb,Tc;

    // 矢量扇区选择  (N=4C+2B+A)
    sector = 0;
    if (v_alpha_beta.Vbeta > 0.0F)
        sector = 1;
    if (((1.73205078F * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0F > 0.0F))
        sector += 2;
    if (((-1.73205078F * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0F > 0.0F))
        sector += 4;

    // 计算矢量作用时间
    switch (sector)
    {
        case 1:
            Tx = (-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            Ty = (1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            break;
        case 2:
            Tx = (1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            Ty = -(1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc);
            break;
        case 3:
            Tx = -((-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            Ty = 1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc;
            break;
        case 4:
            Tx = -(1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc);
            Ty = (-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            break;
        case 5:
            Tx = 1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc;
            Ty = -((1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            break;
        case 6:
            Tx = -((1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            Ty = -((-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            break;
        default:
            Tx = 0.0f;
            Ty = 0.0f;
            break;
    }

    // 过调制
    T = Tx + Ty;
    if (T > Tpwm)
    {
//        Tx /= T;
//        Ty /= T;
		 Tx = Tx * Tpwm / T;  
		 Ty = Ty * Tpwm / T;
    }
    /* 【已知BUG，待修复】过调制限幅算法错误：
       Tx、Ty 是时间量纲（前面乘过 Tpwm/Udc），此处除以 T 后 Tx+Ty 被归一化到 1，
       而不是 Tpwm，导致后续 Ta = (Tpwm - 1)/4 完全错误，过调制一旦触发输出即异常。
       正确写法应为：Tx = Tx * Tpwm / T;  Ty = Ty * Tpwm / T;
       （等比例压缩，使 Tx+Ty = Tpwm，保持电压矢量方向不变、幅值限到六边形边界） */

    // 扇区切换时间点
    Ta = (Tpwm - (Tx + Ty)) / 4.0F;
    Tb = Tx / 2.0F + Ta;
    Tc = Ty / 2.0F + Tb;

    switch (sector)
    {
        case 1:
            rtY.Tcmp1 = Tb;
            rtY.Tcmp2 = Ta;
            rtY.Tcmp3 = Tc;
            break;
        case 2:
            rtY.Tcmp1 = Ta;
            rtY.Tcmp2 = Tc;
            rtY.Tcmp3 = Tb;
            break;
        case 3:
            rtY.Tcmp1 = Ta;
            rtY.Tcmp2 = Tb;
            rtY.Tcmp3 = Tc;
            break;
        case 4:
            rtY.Tcmp1 = Tc;
            rtY.Tcmp2 = Tb;
            rtY.Tcmp3 = Ta;
            break;
        case 5:
            rtY.Tcmp1 = Tc;
            rtY.Tcmp2 = Ta;
            rtY.Tcmp3 = Tb;
            break;
        case 6:
            rtY.Tcmp1 = Tb;
            rtY.Tcmp2 = Tc;
            rtY.Tcmp3 = Ta;
            break;
    }
}

/*
@function: Clarke变换
@para: current_abc: 三相电流结构体
@para: current_alpha_beta: Clarke变换输出结构体
@brief: Clarke变换函数，三相120°电流转换为两相90°电流
*/
void Clarke_Transf(Current_ABC_DEF current_abc, Current_Alpha_Beta_DEF* current_alpha_beta)
{
    current_alpha_beta->Ialpha = current_abc.Ia;
    current_alpha_beta->Ibeta  = (current_abc.Ia + 2.0f * current_abc.Ib) / 1.73205078f;
}

/*
@function: Park变换
@para: current_alpha_beta: Alpha-Beta 电流结构体
@para: 当前电角度的cos_sin: 三角函数结构体
@para: current_id_iq: Id-Iq 输出电流结构体
@brief: Park变换函数，两相90°电流转换为直交轴电流
*/
void Park_Transf(Current_Alpha_Beta_DEF current_alpha_beta, Transf_Cos_Sin_DEF cos_sin, Current_Id_Iq_DEF* current_id_iq)
{
    current_id_iq->Id = cos_sin.Cos * current_alpha_beta.Ialpha + cos_sin.Sin * current_alpha_beta.Ibeta;
    current_id_iq->Iq = -cos_sin.Sin * current_alpha_beta.Ialpha + cos_sin.Cos * current_alpha_beta.Ibeta;
}

