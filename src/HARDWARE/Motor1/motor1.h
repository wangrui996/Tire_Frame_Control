#ifndef __MOTOR1_H
#define __MOTOR1_H 

#include "sys.h"
#include "stdlib.h"	

//˿�ܲ������������ 	
//�޸�����:2020/03/29
//�汾��V1.0

/**********˿�������� �˿ڶ��� **************
//DRIVER1_DIR   PE3   ˿�ܷ���
//DRIVER1_OE    PE4   ˿��������ʹ��
//STEP_PULSE1   PE11(TIM1_CH2)  ˿������������
******************************************/

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

#define DRIVER1_DIR   PEout(3) // ˿�ܵ����ת����
#define DRIVER1_OE    PEout(4) // ˿�ܵ��ʹ�ܽ� �͵�ƽ��Ч 

#define RCR_VAL1      0       //ÿ������RCR_VAL+1���Σ��ж�һ�Σ����ֵ��0~255�����ô�һЩ���Խ����ж�Ƶ��


#define CW1                                   0  //��ת
#define CCW1                                  1  //��ת

#define TRUE 1
#define FALSE 0

//�ٶ�����״̬
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

//ϵͳ״̬
struct GLOBAL_FLAGS1 {
  //�����������������ʱ��ֵΪ1
  unsigned char running:1;
  //�����ڽ��յ�����ʱ��ֵΪ1
  unsigned char cmd:1;
  //���������������ʱ,ֵΪ1
  unsigned char out_ena:1;
};

extern struct GLOBAL_FLAGS1 status1;
extern int stepPosition1;
unsigned static int a1;
unsigned static int d1;

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
} speedRampData1;

#define DIR(a)	if (a == CW)	\
					GPIO_ResetBits(GPIOE,GPIO_Pin_5);\
					else		\
					GPIO_SetBits(GPIOE,GPIO_Pin_5)


/* �������� ------------------------------------------------------------------*/
									
void Driver1_Init(void);  //�����������ź��߳�ʼ�� 
void TIM1_OPM_RCR_Init(u16 arr,u16 psc);//TIM1_CH2 ���������+�ظ��������ܳ�ʼ��
void TIM1_Startup(u32 frequency);   //������ʱ��1(��ʱ�����ú�����Ƶ��)
//void Motor1_Ctrl(u8 Dir , float Frequency);
void MSD_ENA1(FunctionalState NewState);
void MSD_Move1(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);

#endif

