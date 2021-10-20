#ifndef __ADC_H
#define __ADC_H 
#include "sys.h"

//ADC 压力采集	锂电池电压采集
//修改日期:2020/03/24
//版本：V1.0

///********** 端口定义 **************
// PA5 压力采集口
// PA4 锂电池电压采集口

void Adc_Init(void); 				//ADC通道初始化


u16  Get_Adc1(u8 ch); 				//获得ADC1值-----压力采样

u16 Get_Adc1_Average(u8 ch,u8 times);//获得ADC1的times次采样平均值-------压力采样平均值   

u16  Get_Adc2(u8 ch); 				//获得ADC2值-----锂电池电压采样

u16 Get_Adc2_Average(u8 ch,u8 times);//获得ADC2的times次采样平均值-------锂电池电压采样平均值  

/*****ADC 压力采集   ADC锂电池电压采集********/




#endif
