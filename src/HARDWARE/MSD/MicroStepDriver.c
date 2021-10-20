/**
  ******************************************************************************
  * @file    MicroStepDriver.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   MSD��������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����STM32 F103-ָ���� ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "MicroStepDriver.h" 
#include <stdio.h>
#include <math.h>
#include "usart.h"

//ϵͳ�Ӽ��ٲ���
speedRampData srd;
//��¼���������λ��
int stepPosition = 0;
//ϵͳ���������״̬
struct GLOBAL_FLAGS status = {FALSE, FALSE,TRUE};

/**

  * @brief  ��ʱ���ж����ȼ�����

  * @param  ��

  * @retval ��

  */
static void TIM_NVIC_Config(void)

{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // �����ж���Ϊ0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
	// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = MSD_PULSE_TIM_IRQ; 	
	// ������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	// ���������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
/**

  * @brief  ��ʼ����������õ�������

  * @param  ��

  * @retval ��

  */
static void MSD_GPIO_Config(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //���������� GPIO ��ʼ��
	RCC_AHB1PeriphClockCmd(MSD_PULSE_GPIO_CLK,ENABLE); //ʹ��GPIOAʱ��
    //RCC_APB2PeriphClockCmd(MSD_PULSE_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  MSD_PULSE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MSD_PULSE_PORT, &GPIO_InitStructure);
    
    //���������� GPIO ��ʼ��
    RCC_AHB1PeriphClockCmd(MSD_DIR_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  MSD_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MSD_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MSD_DIR_PORT,MSD_DIR_PIN);
    
    //���ʹ����� GPIO ��ʼ��
    RCC_AHB1PeriphClockCmd(MSD_ENA_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  MSD_ENA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MSD_ENA_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MSD_ENA_PORT,MSD_ENA_PIN);
}


///*
// * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
// * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
// * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            ����
// *  TIM_CounterMode		   TIMx,x[6,7]û�У���������
// *  TIM_Period               ����
// *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
// *}TIM_TimeBaseInitTypeDef; 
// *-----------------------------------------------------------------------------
// */

/* ----------------   PWM�ź� ���ں�ռ�ձȵļ���--------------- */
// ARR ���Զ���װ�ؼĴ�����ֵ
// CLK_cnt����������ʱ�ӣ����� Fck_int / (psc+1) = 72M/(psc+1)
// PWM �źŵ����� T = (ARR+1) * (1/CLK_cnt) = (ARR+1)*(PSC+1) / 72M
// ռ�ձ�P=CCR/(ARR+1)

static void TIM_Mode_Config(void)
{
  // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	MSD_PULSE_TIM_APBxClock_FUN(MSD_PULSE_TIM_CLK,ENABLE);

    /*--------------------ʱ���ṹ���ʼ��-------------------------*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    // �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1�����ں����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Period=MSD_PULSE_TIM_PERIOD;	
	// ����CNT��������ʱ�� = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= MSD_PULSE_TIM_PSC;	
	// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// �ظ���������ֵ�����ֵΪ255
	//TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��
	TIM_TimeBaseInit(MSD_PULSE_TIM, &TIM_TimeBaseStructure);

	/*--------------------����ȽϽṹ���ʼ��-------------------*/		
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// ����ΪPWMģʽ2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	// ���ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// �����������
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 
	// ����ռ�ձȴ�С
	TIM_OCInitStructure.TIM_Pulse = MSD_PULSE_TIM_PERIOD/2;
	// ���ͨ����ƽ��������
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	// ���ͨ�����е�ƽ��������
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    
	MSD_PULSE_OCx_Init(MSD_PULSE_TIM, &TIM_OCInitStructure);
    //ʹ��TIM1_CH1Ԥװ�ؼĴ���
	MSD_PULSE_OCx_PreloadConfig(MSD_PULSE_TIM, TIM_OCPreload_Enable);
    //ʹ��TIM1Ԥװ�ؼĴ���
    TIM_ARRPreloadConfig(MSD_PULSE_TIM, ENABLE); 
    
    //�����ж�Դ��ֻ�����ʱ���ж�
    TIM_UpdateRequestConfig(MSD_PULSE_TIM,TIM_UpdateSource_Regular);
	// ����жϱ�־λ
	TIM_ClearITPendingBit(MSD_PULSE_TIM, TIM_IT_Update);
    // ʹ���ж�
    TIM_ITConfig(MSD_PULSE_TIM, TIM_IT_Update, ENABLE);
	// ʹ�ܼ�����
	TIM_Cmd(MSD_PULSE_TIM, DISABLE);
}
/**

  * @brief  ��ʼ�������ص�����

  * @param  ��

  * @retval ��

  */
void MSD_Init(void)
{
    MSD_GPIO_Config();
    
    TIM_NVIC_Config();

    TIM_Mode_Config();    
}
/**

  * @brief  ����������ֹͣ

  * @param  NewState��ʹ�ܻ��߽�ֹ

  * @retval ��

  */
void MSD_ENA(FunctionalState NewState)
{
    if(NewState)
    {
      //ENAʧ�ܣ���ֹ���������
      GPIO_SetBits(MSD_ENA_PORT,MSD_ENA_PIN);
      //����ֹͣ��־λΪ��
      status.out_ena = FALSE; 
      printf("\n\r��������ֹ������ѻ�״̬����ʱ���Ϊ�ޱ�������״̬�������ֶ���ת���");        
    }
    else
    {
      //ENAʹ��
      GPIO_ResetBits(MSD_ENA_PORT,MSD_ENA_PIN);
      //����ֹͣ��־λΪ��
      status.out_ena = TRUE; 
      printf("\n\r�������ָ����У���ʱ���Ϊ��������״̬����ʱ����ָ������������Ƶ��");         
    }
    
}
/*! \brief �Ը����Ĳ����ƶ��������
 *  ͨ��������ٵ�����ٶȣ��Ը����Ĳ�����ʼ����
 *  ������ٶȺͼ��ٶȺ�С������������ƶ���������û�ﵽ����ٶȾ�Ҫ��ʼ����
 *  \param step   �ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
 *  \param accel  ���ٶ�,���ȡֵΪ100��ʵ��ֵΪ100*0.01*rad/sec^2=1rad/sec^2
 *  \param decel  ���ٶ�,���ȡֵΪ100��ʵ��ֵΪ100*0.01*rad/sec^2=1rad/sec^2
 *  \param speed  ����ٶ�,���ȡֵΪ100��ʵ��ֵΪ100*0.01*rad/sec=1rad/sec
 */
void MSD_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
    //�ﵽ����ٶ�ʱ�Ĳ���.
    unsigned int max_s_lim;
    //���뿪ʼ���ٵĲ���(�����û���ٵ�������ٶ�ʱ)��
    unsigned int accel_lim;

    // ���ݲ������������жϷ���
    if(step < 0)//��ʱ��
    {
        srd.dir = CCW;
        step = -step;
    }
    else//˳ʱ��
    {
        srd.dir = CW;
    }
    // ����������
    DIR(srd.dir);
    // ���õ��Ϊ���״̬
    //status.out_ena = TRUE;
    
    // ���ֻ�ƶ�һ��
    if(step == 1)
    {
        // ֻ�ƶ�һ��
        srd.accel_count = -1;
        // ����״̬
        srd.run_state = DECEL;
        // ����ʱ
        srd.step_delay = 1000;
        // ���õ��Ϊ����״̬
        status.running = TRUE;
        //���ö�ʱ����װֵ	
        TIM_SetAutoreload(MSD_PULSE_TIM,Pulse_width);
        //����ռ�ձ�Ϊ50%	
        TIM_SetCompare2(MSD_PULSE_TIM,Pulse_width>>1);
        //ʹ�ܶ�ʱ��	      
        TIM_Cmd(MSD_PULSE_TIM, ENABLE); 
     }
    // ������Ϊ����ƶ�
    else if(step != 0)
    {
    // ���ǵ��������û��ֲ�����ϸ�ļ��㼰�Ƶ�����

    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    // min_delay = (alpha / tt)/ w
    srd.min_delay = A_T_x100 / speed;

    // ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.01rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100;

    // ������ٲ�֮��ﵽ����ٶȵ�����
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);
    // ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
    // ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
    if(max_s_lim == 0)
    {
        max_s_lim = 1;
    }

    // ������ٲ�֮�����Ǳ��뿪ʼ����
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = ((long)step*decel) / (accel+decel);
    // ���Ǳ����������1�����ܲ��ܿ�ʼ����.
    if(accel_lim == 0)
    {
        accel_lim = 1;
    }
    // ʹ�������������ǿ��Լ������һ�ο�ʼ���ٵ�λ��
    //srd.decel_valΪ����
    if(accel_lim <= max_s_lim)
    {
        srd.decel_val = accel_lim - step;
    }
    else
    {
        srd.decel_val = -(long)(max_s_lim*accel/decel);
    }
    // ��ֻʣ��һ�����Ǳ������
    if(srd.decel_val == 0)
    {
        srd.decel_val = -1;
    }

    // ���㿪ʼ����ʱ�Ĳ���
    srd.decel_start = step + srd.decel_val;

    // �������ٶȺ��������ǾͲ���Ҫ���м����˶�
    if(srd.step_delay <= srd.min_delay)
    {
        srd.step_delay = srd.min_delay;
        srd.run_state = RUN;
    }
    else
   {
        srd.run_state = ACCEL;
    }

    // ��λ���ٶȼ���ֵ
    srd.accel_count = 0;
    status.running = TRUE;
    //���ö�ʱ����װֵ	
    TIM_SetAutoreload(MSD_PULSE_TIM,Pulse_width);
    //����ռ�ձ�Ϊ50%	
    TIM_SetCompare2(MSD_PULSE_TIM,Pulse_width>>1);
    //ʹ�ܶ�ʱ��	      
    TIM_Cmd(MSD_PULSE_TIM, ENABLE); 
    }
}

/**

  * @brief  �����˶������жϲ������������λ��

  * @param  inc �˶�����

  * @retval ��

  */
void MSD_StepCounter(signed char inc)
{
  //���ݷ����жϵ��λ��
  if(inc == CCW)
  {
    stepPosition--;
  }
  else
  {
    stepPosition++;
  }
}
/**

  * @brief  �������嶨ʱ�����ж���Ӧ����ÿ��һ����������˶�״̬

  * @param  ��

  * @retval ��

  */
void MSD_PULSE_TIM_IRQHandler(void)  //�жϳ���
{
  // ������һ����ʱ����
  unsigned int new_step_delay;
  // ���ٹ��������һ����ʱ.
  static int last_accel_delay;
  // �ƶ�����������
  static unsigned int step_count = 0;
  // ��¼new_step_delay�е������������һ������ľ���
  static signed int rest = 0;

if (TIM_GetITStatus(MSD_PULSE_TIM, TIM_IT_Update) != RESET)
{
    /* Clear MSD_PULSE_TIM Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(MSD_PULSE_TIM, TIM_IT_Update);

    MSD_PULSE_TIM->CCR4=srd.step_delay >> 1;//���ڵ�һ��    //ռ�ձ�
    MSD_PULSE_TIM->ARR=srd.step_delay;   //�����Զ���װ��ֵ
    //�����ֹ����������ֹͣ�˶�
    if(status.out_ena != TRUE)
    {
        srd.run_state = STOP;
    }
  switch(srd.run_state) {
    case STOP:
      step_count = 0;
      rest = 0;
      MSD_PULSE_TIM->CCER &= ~(1<<12); //��ֹ���
      TIM_Cmd(MSD_PULSE_TIM, DISABLE);
      status.running = FALSE;
      break;

    case ACCEL:
      MSD_PULSE_TIM->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter(srd.dir);
      step_count++;
      srd.accel_count++;
      new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) 
                       + rest)/(4 * srd.accel_count + 1));
      rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
      //����ǹ�Ӧ�ÿ�ʼ����
      if(step_count >= srd.decel_start) {
        srd.accel_count = srd.decel_val;
        srd.run_state = DECEL;
      }
      //����Ƿ񵽴�����������ٶ�
      else if(new_step_delay <= srd.min_delay) {
        last_accel_delay = new_step_delay;
        new_step_delay = srd.min_delay;
        rest = 0;
        srd.run_state = RUN;
      }
      break;

    case RUN:
      MSD_PULSE_TIM->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter(srd.dir);
      step_count++;
      new_step_delay = srd.min_delay;
      //����Ƿ���Ҫ��ʼ����
      if(step_count >= srd.decel_start) {
        srd.accel_count = srd.decel_val;
        //�����һ�μ��ٵ���ʱ��Ϊ��ʼ���ٵ���ʱ
        new_step_delay = last_accel_delay;
        srd.run_state = DECEL;
      }
      break;

    case DECEL:
      MSD_PULSE_TIM->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter(srd.dir);
      step_count++;
      srd.accel_count++;
      new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) 
                       + rest)/(4 * srd.accel_count + 1));
      rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
      //����Ƿ�Ϊ���һ��
      if(srd.accel_count >= 0){
        srd.run_state = STOP;
      }
      break;
  }
  srd.step_delay = new_step_delay;
  }
}

/*********************************************END OF FILE**********************/