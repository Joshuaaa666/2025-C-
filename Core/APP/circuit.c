#include "main.h"
#include "adc.h"

#define R 					0.2		//系统串联电阻
#define R_offset			0.11		//系统电路偏置电阻
#define R_to_GND			68000*2 	//接地电阻
#define Filter_Tolerance	0.03		//滤波容忍度，越小滤波能力越大


/*********************初始化函数*************************/
void Circuit_Init()
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
}


/*********************获取单次的系统总电流*************************/		//单位：毫安
double Circuit_GetSingleCurrent()
{
	int value1=0;
	int value2=0;
	double volt1=0;
	double volt2=0;
	double I0=0,I1=0;
	HAL_ADC_Start(&hadc1);				//开启ADC
	HAL_ADC_Start(&hadc2);				//开启ADC
	HAL_Delay(1);						//延时等待ADC稳定
	value1 = HAL_ADC_GetValue(&hadc1);	//获取ADC值
	value2 = HAL_ADC_GetValue(&hadc2);	//获取ADC值
	volt1 = value1/65535.0 *3.3;		//转换成电压值
	volt2 = value2/65535.0 *3.3;		//转换成电压值
	if(volt1>volt2)						//判断两个ADC通道的极性，防反接
	{
		I0 = ( volt1*2 - volt2*2 ) / ( R + R_offset );	//计算系统主路径上的电流
		I1 = volt1 / R_to_GND;			//计算分压电路上的电流
	}
	else
	{
		I0 = ( volt2*2 - volt1*2 ) / ( R + R_offset );
		I1 = volt2 / R_to_GND;		
	}
	HAL_ADC_Stop(&hadc1);				//关闭ADC
	HAL_ADC_Stop(&hadc2);  
	return (I0+I1)*1000;				//转换单位
}


/*********************获取滤波后的系统总电流*************************/			//单位：毫安
double Circuit_GetMultiCurrent()
{
	double I=0,I_ref;
	int referrence_times=50;
	int repeat_times=50;
	int skip_times=0;
	for(int i=0;i<referrence_times;i++)	//提前读数，计算参考稳定值
	{
		I_ref += Circuit_GetSingleCurrent();
	}
	I_ref = I_ref / referrence_times;
	
	for(int i=0;i<repeat_times;i++)		//计算平均电压
	{
		double temp;
		temp = Circuit_GetSingleCurrent();
		if( temp/I_ref > 1+Filter_Tolerance || temp/I_ref< 1 - Filter_Tolerance)	//电压跳变作滤波处理
		{
			skip_times++;
			continue;
		}
		I += temp;
	}
	I = repeat_times!=skip_times ? ( I / ( repeat_times-skip_times)) : I_ref;
	return I;
}


/*********************获取单次的系统总功率*************************/		//单位：毫瓦	
double Circuit_GetSinglePower()
{
	int value1=0;
	int value2=0;
	double volt1=0;
	double volt2=0;
	double I0=0,I1=0;
	double I,U=5.0,P;
	HAL_ADC_Start(&hadc1);				//开启ADC
	HAL_ADC_Start(&hadc2);				//开启ADC
	HAL_Delay(1);						//延时等待ADC稳定
	value1 = HAL_ADC_GetValue(&hadc1);	//获取ADC值
	value2 = HAL_ADC_GetValue(&hadc2);	//获取ADC值
	volt1 = value1/65535.0 *3.3;		//转换成电压值
	volt2 = value2/65535.0 *3.3;		//转换成电压值
	if(volt1>volt2)						//判断两个ADC通道的极性，防反接
	{
		I0 = ( volt1*2 - volt2*2 ) / ( R + R_offset );	//计算系统主路径上的电流
		I1 = volt1 / R_to_GND;			//计算分压电路上的电流
		//U = volt1*2;
	}
	else
	{
		I0 = ( volt2*2 - volt1*2 ) / ( R + R_offset );
		I1 = volt2 / R_to_GND;		
		//U = volt2*2;
	}
	I = I0 + I1;
	P = U * I;
	HAL_ADC_Stop(&hadc1);				//关闭ADC
	HAL_ADC_Stop(&hadc2);  
	return P*1000;						//转换单位
}


/*********************获取滤波后的系统总功率*************************/		//单位：毫瓦	
double Circuit_GetMultiPower()
{
	double P=0,P_ref;
	int referrence_times=50;
	int repeat_times=50;
	int skip_times=0;
	for(int i=0;i<referrence_times;i++)	//提前读数，计算参考稳定值
	{
		P_ref += Circuit_GetSinglePower();
	}
	P_ref = P_ref / referrence_times;
	
	for(int i=0;i<repeat_times;i++)		//计算平均功率
	{
		double temp;
		temp = Circuit_GetSinglePower();
		if( temp/P_ref > 1 + Filter_Tolerance || temp/P_ref< 1 - Filter_Tolerance)	//功率跳变作滤波处理
		{
			skip_times++;
			continue;
		}
		P += temp;
	}
	P = repeat_times!=skip_times ? ( P / ( repeat_times-skip_times)) : P_ref;
	return P;
}


















