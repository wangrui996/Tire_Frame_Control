#ifndef __ADC_H
#define __ADC_H 
#include "sys.h"

//ADC ѹ���ɼ�	﮵�ص�ѹ�ɼ�
//�޸�����:2020/03/24
//�汾��V1.0

///********** �˿ڶ��� **************
// PA5 ѹ���ɼ���
// PA4 ﮵�ص�ѹ�ɼ���

void Adc_Init(void); 				//ADCͨ����ʼ��


u16  Get_Adc1(u8 ch); 				//���ADC1ֵ-----ѹ������

u16 Get_Adc1_Average(u8 ch,u8 times);//���ADC1��times�β���ƽ��ֵ-------ѹ������ƽ��ֵ   

u16  Get_Adc2(u8 ch); 				//���ADC2ֵ-----﮵�ص�ѹ����

u16 Get_Adc2_Average(u8 ch,u8 times);//���ADC2��times�β���ƽ��ֵ-------﮵�ص�ѹ����ƽ��ֵ  

/*****ADC ѹ���ɼ�   ADC﮵�ص�ѹ�ɼ�********/




#endif
