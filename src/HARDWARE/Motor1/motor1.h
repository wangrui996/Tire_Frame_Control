#ifndef __MOTOR1_H
#define __MOTOR1_H 

#include "sys.h"
#include "stdlib.h"	

//丝杠步进电机驱动器 	
//修改日期:2020/03/29
//版本：V1.0

/**********丝杠驱动器 端口定义 **************
//DRIVER1_DIR   PE3   丝杠方向
//DRIVER1_OE    PE4   丝杠驱动器使能
//STEP_PULSE1   PE11(TIM1_CH2)  丝杠驱动器脉冲
******************************************/

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/

#define DRIVER1_DIR   PEout(3) // 丝杠电机旋转方向
#define DRIVER1_OE    PEout(4) // 丝杠电机使能脚 低电平有效 

#define RCR_VAL1      0       //每计数（RCR_VAL+1）次，中断一次，这个值（0~255）设置大一些可以降低中断频率


#define CW1                                   0  //正转
#define CCW1                                  1  //正转

#define TRUE 1
#define FALSE 0

//速度曲线状态
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

//系统状态
struct GLOBAL_FLAGS1 {
  //当步进电机正在运行时，值为1
  unsigned char running:1;
  //当串口接收到数据时，值为1
  unsigned char cmd:1;
  //当驱动器正常输出时,值为1
  unsigned char out_ena:1;
};

extern struct GLOBAL_FLAGS1 status1;
extern int stepPosition1;
unsigned static int a1;
unsigned static int d1;

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
} speedRampData1;

#define DIR(a)	if (a == CW)	\
					GPIO_ResetBits(GPIOE,GPIO_Pin_5);\
					else		\
					GPIO_SetBits(GPIOE,GPIO_Pin_5)


/* 函数声明 ------------------------------------------------------------------*/
									
void Driver1_Init(void);  //驱动器控制信号线初始化 
void TIM1_OPM_RCR_Init(u16 arr,u16 psc);//TIM1_CH2 单脉冲输出+重复计数功能初始化
void TIM1_Startup(u32 frequency);   //启动定时器1(此时即设置好脉冲频率)
//void Motor1_Ctrl(u8 Dir , float Frequency);
void MSD_ENA1(FunctionalState NewState);
void MSD_Move1(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);

#endif

