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
//步进电机驱动代码	   					  
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

#define ADD_CNT			10	//加10次速
#define T_ADD				200//ms
#define TIMER_F			1000000//1Mhz
#define OBJ_F				1500//hz
#define START_F			50//hz
#define ADD_SPEED		(OBJ_F-START_F)/ADD_CNT
#define NEED_PLUSE	(START_F*ADD_CNT+ADD_SPEED*45)*T_ADD/1000*2 //允许匀加速运动需要的脉冲数  
																																//45是加速次数的累加和 每次修改加速次数需要修改这个地方




#if STEP_MOTOR

typedef struct{
	u16 pluse_ObjSend;	//要发送的最大脉冲数
	u16	pluse_Current;	//当前已发送脉冲
	u16 speed_Current;	//当前速度
	u16 pluse_send;		//当前已经发送的脉冲
	u8	Add_Cnt;		//已经加速N次
	u8	Add_Minus;		//加速减速标记
	u8	Finish;			//加减速完成标记
	u16	Arr_tim3;		//要写入TIM3->ARR寄存器的值
	u16	Arr_tim4_start;	//TIM4最初值
	u8	adjval;			//校准的脉冲值
	u8	flag;			//标记位，	bit0 表示 是否需要匀加速		1:YES 0:NO
									//			bit1 表示 是上升还是下降		1:UP 0:DOWN
									//			bit8 表示 是否到达最低限位 	1:YES 0:NO
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
