#ifndef __MOTOR2_H
#define __MOTOR2_H 

#include "sys.h"
#include "stdlib.h"	

//���̲������������ ���Դ���	
//�޸�����:2020/02/20
//�汾��V1.0

/**********���������� �˿ڶ��� **************
//DRIVER1_DIR   PE5 (DCMI_D6)   ���̷���
//DRIVER1_OE    PE6 (DCMI_D7)   ����������ʹ��
//STEP_PULSE1   PC7 (TIM8_CH2  DCMI_D1)
******************************************/
#define DRIVER2_DIR   PEout(5) // ���̵����ת����
#define DRIVER2_OE    PEout(6) // ���̵��ʹ�ܽ� �͵�ƽ��Ч 

#define RCR_VAL      0        //ÿ������RCR_VAL+1���Σ��ж�һ�Σ����ֵ��0~255�����ô�һЩ���Խ����ж�Ƶ��


#define CW  0
#define CCW 1

#define TRUE 1
#define FALSE 0

#define Pulse_width 20

/*
#define T1_FREQ 1000000     //��ʱ��Ƶ��
#define FSPR    200         //���������Ȧ����
#define SPR     (FSPR*10)  //10ϸ�ֵĲ���
// ��ѧ������ ����MSD_Move�����ļ򻯼���
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
 */ 
 
//�ٶ�����״̬
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3


//ϵͳ״̬
struct GLOBAL_FLAGS {
  //�����������������ʱ��ֵΪ1
  unsigned char running:1;
  //�����ڽ��յ�����ʱ��ֵΪ1
  unsigned char cmd:1;
  //���������������ʱ,ֵΪ1
  unsigned char out_ena:1;
};

extern struct GLOBAL_FLAGS status;
extern int stepPosition;
unsigned static int a;
unsigned static int d;

typedef struct {
  //�������״̬
  unsigned char run_state : 3;
  //������з���
  unsigned char dir : 1;
  //��һ��������ʱ���ڣ�����ʱΪ���ٶ�����
  unsigned int step_delay;
  //��ʼ���ٵ�λ��
  unsigned int decel_start;
  //���پ���
  signed int decel_val;
  //��С��ʱ��������ٶȣ�
  signed int min_delay;
  //���ٻ��߼��ټ�����
  signed int accel_count;
  //���ټ�����
  signed int decel_count;
} speedRampData;

#define DIR(a)	if (a == CW)	\
					GPIO_ResetBits(GPIOE,GPIO_Pin_5);\
					else		\
					GPIO_SetBits(GPIOE,GPIO_Pin_5)


					
void Driver_Init(void);  //�����������ź��߳�ʼ�� 
void TIM8_OPM_RCR_Init(u16 arr,u16 psc);//TIM8_CH2 ���������+�ظ��������ܳ�ʼ��
void TIM8_Startup(u32 frequency);   //������ʱ��8(��ʱ�����ú�����Ƶ��)
//void Locate_Rle(long num,u32 frequency,DIR_Type dir); //��Զ�λ����
//void Locate_Abs(long num,u32 frequency);//���Զ�λ����

void MSD_ENA(FunctionalState NewState);
void MSD_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);

#endif

