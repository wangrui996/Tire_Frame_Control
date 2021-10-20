#ifndef __MOTOR2_H
#define __MOTOR2_H 

#include "sys.h"
#include "stdlib.h"	

//底盘步进电机驱动器 测试代码	
//修改日期:2020/02/20
//版本：V1.0

/**********底盘驱动器 端口定义 **************
//DRIVER1_DIR   PE5 (DCMI_D6)   底盘方向
//DRIVER1_OE    PE6 (DCMI_D7)   底盘驱动器使能
//STEP_PULSE1   PC7 (TIM8_CH2  DCMI_D1)
******************************************/
#define DRIVER2_DIR   PEout(5) // 底盘电机旋转方向
#define DRIVER2_OE    PEout(6) // 底盘电机使能脚 低电平有效 

#define RCR_VAL      0        //每计数（RCR_VAL+1）次，中断一次，这个值（0~255）设置大一些可以降低中断频率


#define CW  0
#define CCW 1

#define TRUE 1
#define FALSE 0

#define Pulse_width 20

/*
#define T1_FREQ 1000000     //定时器频率
#define FSPR    200         //步进电机单圈步数
#define SPR     (FSPR*10)  //10细分的步数
// 数学常数。 用于MSD_Move函数的简化计算
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
 */ 
 
//速度曲线状态
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3


//系统状态
struct GLOBAL_FLAGS {
  //当步进电机正在运行时，值为1
  unsigned char running:1;
  //当串口接收到数据时，值为1
  unsigned char cmd:1;
  //当驱动器正常输出时,值为1
  unsigned char out_ena:1;
};

extern struct GLOBAL_FLAGS status;
extern int stepPosition;
unsigned static int a;
unsigned static int d;

typedef struct {
  //电机运行状态
  unsigned char run_state : 3;
  //电机运行方向
  unsigned char dir : 1;
  //下一个脉冲延时周期，启动时为加速度速率
  unsigned int step_delay;
  //开始减速的位置
  unsigned int decel_start;
  //减速距离
  signed int decel_val;
  //最小延时（即最大速度）
  signed int min_delay;
  //加速或者减速计数器
  signed int accel_count;
  //减速计数器
  signed int decel_count;
} speedRampData;

#define DIR(a)	if (a == CW)	\
					GPIO_ResetBits(GPIOE,GPIO_Pin_5);\
					else		\
					GPIO_SetBits(GPIOE,GPIO_Pin_5)


					
void Driver_Init(void);  //驱动器控制信号线初始化 
void TIM8_OPM_RCR_Init(u16 arr,u16 psc);//TIM8_CH2 单脉冲输出+重复计数功能初始化
void TIM8_Startup(u32 frequency);   //启动定时器8(此时即设置好脉冲频率)
//void Locate_Rle(long num,u32 frequency,DIR_Type dir); //相对定位函数
//void Locate_Abs(long num,u32 frequency);//绝对定位函数

void MSD_ENA(FunctionalState NewState);
void MSD_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);

#endif

