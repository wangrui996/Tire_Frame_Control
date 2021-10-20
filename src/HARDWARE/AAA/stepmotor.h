#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H	 
#include "sys.h"
#include "led.h"
#include "usart2.h"
#include "delay.h"
#include "DCMotorDrive.h"
#include "GearboxMotorDrive.h"
#include "key.h"
#include "string.h"

//////////////////////////////////////////////////////////////////////////////////	 
//���������������	   					  
////////////////////////////////////////////////////////////////////////////////// 
#define DIR		PBout(11)
#define DE		PBout(10)

#define DR		PEout(15)// PE6
#define MF		PEout(14)// PE6

#define UP			0
#define DOWN		1

#define STEP_MOTOR		1
#define	ADD				1
#define	MINUS			0
#define STOP			0xFF

//////////////////////////////////////////////
#define LOWEST_SPEED	10000
#define	MAX_SPEED		1000

//#define	ADD_SPEED		(LOWEST_SPEED-MAX_SPEED)/ADD_CNT
/////////////////////////////

#define ADD_CNT			10	//��10����
#define T_ADD				200//ms
#define TIMER_F			1000000//1Mhz
#define OBJ_F				1500//hz
#define START_F			50//hz
#define ADD_SPEED		(OBJ_F-START_F)/ADD_CNT
#define NEED_PLUSE	(START_F*ADD_CNT+ADD_SPEED*45)*T_ADD/1000*2 //�����ȼ����˶���Ҫ��������  
																																//45�Ǽ��ٴ������ۼӺ� ÿ���޸ļ��ٴ�����Ҫ�޸�����ط�




#if STEP_MOTOR

typedef struct{
	u16 pluse_ObjSend;	//Ҫ���͵����������
	u16	pluse_Current;	//��ǰ�ѷ�������
	u16 speed_Current;	//��ǰ�ٶ�
	u16 pluse_send;		//��ǰ�Ѿ����͵�����
	u8	Add_Cnt;		//�Ѿ�����N��
	u8	Add_Minus;		//���ټ��ٱ��
	u8	Finish;			//�Ӽ�����ɱ��
	u16	Arr_tim3;		//Ҫд��TIM3->ARR�Ĵ�����ֵ
	u16	Arr_tim4_start;	//TIM4���ֵ
	u8	adjval;			//У׼������ֵ
	u8	flag;			//���λ��	bit0 ��ʾ �Ƿ���Ҫ�ȼ���		1:YES 0:NO
									//			bit1 ��ʾ �����������½�		1:UP 0:DOWN
									//			bit8 ��ʾ �Ƿ񵽴������λ 	1:YES 0:NO
}StepMotorAddSpeed; 



void TIM4_Int_Init(u16 arr,u16 psc);
void MotorInit(void);		 			
void MotorStart(u8 UpORDown,u16 speed,u16 tim3_cnt_int);
void MotorStop(void);
void MotorStartPluse(u32 pluse,u8 UpORDown);
void TIM3_Int_Init(u16 arr);
void MotorAdjTest_X(u8 UpORDown);
void MotorAdjTest_C(u8 UpORDown);

u8 Motor_TO_OBJfloor (u8 outputprofloor);

u8 Motor_TO_OBJwindowTwo (void);
u8 Motor_TO_Lowest(u16 speed,u8 timeout);
u8 Motor_TO_Lowest_N(u16 speed,u8 timeout);

#endif
#endif
