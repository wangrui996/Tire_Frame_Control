#include "motor2.h"
#include "usart.h"
#include "sys.h"
#include "math.h"
#include "rs485.h"

//���̲������������ ���Դ���	
//�޸�����:2020/05/20
//�汾��V2.0

/**********���������� �˿ڶ��� **************
//DRIVER1_DIR   PE5 (DCMI_D6)   ���̷���
//DRIVER1_OE    PE6 (DCMI_D7)   ����������ʹ��
//STEP_PULSE1   PC7 (TIM8_CH2  DCMI_D1)
******************************************/
u8 rcr_remainder;   //�ظ�������������  rcr_remainderΪ0-255
long rcr_integer;	//�ظ�������������
u8 is_rcr_finish=1; //���巢���Ƿ����   ���Ϊ1
long target_pos=0;  //Ŀ��λ��-��Ŀ����������
long current_pos=0; //��ǰλ��-����ǰ��������
//DIR_Type motor_dir=CW;//CW��Ϊ1��1˳ʱ��

//ϵͳ�Ӽ��ٲ���
speedRampData srd;
//��¼���������λ��
int stepPosition = 0;
//ϵͳ���������״̬
struct GLOBAL_FLAGS status = {FALSE, FALSE,TRUE};
unsigned static int a=0;
unsigned static int d=0;

/************** �����������ź��߳�ʼ�� ****************/
void Driver_Init(void)
{
     /***** ���������������ź��߳�ʼ��*******/     //PE5 ����  PE6 ʹ��
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //ʹ��GPIOEʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;//DRIVER2_DIR DRIVER2_OE��Ӧ����PE5 PE6
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;       //��ͨ���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      //�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOE, &GPIO_InitStructure);  //��ʼ��GPIOE5
	
	GPIO_SetBits(GPIOE,GPIO_Pin_5); //PE5����� ˳ʱ�뷽��  DRIVER2_DIR
	GPIO_ResetBits(GPIOE,GPIO_Pin_6); //PE6����� ʹ�����  DRIVER2_OE
	
}	
/***********************************************
//TIM8_CH2(PC7) ���������+�ظ��������ܳ�ʼ��
//TIM8 ʱ��Ƶ�� 84*2=168MHz
//arr:�Զ���װֵ
//psc:ʱ��Ԥ��Ƶ��
************************************************/
void TIM8_OPM_RCR_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8ʱ��ʹ��  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTCʱ��	
	
	/* GPIOC7����Ϊ��ʱ��8  */
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;           //TIM8_ch2��ӦIO��GPIOC7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;      //����             Ϊʲô��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ��GPIOC7
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); //GPIOC7����Ϊ��ʱ��8
	
	/* ��ʼ����ʱ��8  */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //����ʱ�ӷָ� TIM_CKD_DIV1=0
	
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //��ʼ��TIM8
	
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);//�����ʱ��TIM8���ж�TIM_IT��־λ
	TIM_UpdateRequestConfig(TIM8,TIM_UpdateSource_Regular); /********* ����ֻ�м��������Ϊ�����ж� ********/
	TIM_SelectOnePulseMode(TIM8,TIM_OPMode_Single);/******* ������ģʽ **********/
	
	/*��ʼ��TIM8��ͨ��2*/
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�,,PWM������ʼ��ƽΪ��
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//�Ƚ����2ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; /****** �Ƚ����2Nʧ��????? *******/
	TIM_OCInitStructure.TIM_Pulse = arr>>1; // PWM�ıȽ�ֵ�����ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIM8��ͨ��2
	
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //TIM8��CH2Ԥװ��ʹ��	 
	TIM_ARRPreloadConfig(TIM8, ENABLE); //ʹ��TIM8��ARR�ϵ�Ԥװ�ؼĴ���
	
	
	/*��ʼ��TIM8�ж�*/
	TIM_ITConfig(TIM8, TIM_IT_Update ,ENABLE);  //TIM8ʹ�ܻ���ʧ��ָ����TIM�жϺ���
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn;  //TIM8�����ж�;  NVIC_IRQChannelȷ������һ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;//��ռ���ȼ�1��  NVIC_IRQChannelPreemptionPriority��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;// ��Ӧ���ȼ�1��        NVIC_IRQChannelSubPriority��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ
	
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8
}


void MSD_ENA(FunctionalState NewState)
{
    if(NewState)
    {
      //ENAʧ�ܣ���ֹ���������
      GPIO_SetBits(GPIOE,GPIO_Pin_6);
      //����ֹͣ��־λΪ��
      status.out_ena = FALSE; 
      printf("\n\r��������ֹ������ѻ�״̬����ʱ���Ϊ�ޱ�������״̬�������ֶ���ת���");        
    }
    else
    {
      //ENAʹ��
      GPIO_ResetBits(GPIOE,GPIO_Pin_6);
      //����ֹͣ��־λΪ��
      status.out_ena = TRUE; 
      printf("\n\r�������ָ����У���ʱ���Ϊ��������״̬����ʱ����ָ������������Ƶ��");         
    }
    
}

/*! \brief �Ը������������ƶ��������
 *  ͨ��������ٵ�����ٶȣ��Ը�������������ʼ����
 *  ������ٶȺͼ��ٶȺ�С������������ƶ���������û�ﵽ����ٶȾ�Ҫ��ʼ����
 *  \param step   �ƶ��������� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
 *  \param accel  ���ٶ�,���ȡֵΪ10��10Hz/����
 *  \param decel  ���ٶ�,���ȡֵΪ10��10Hz/����
 *  \param speed  ���Ƶ��,���ȡֵΪ100,���Ƶ��100Hz
 */
void MSD_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
	//�ﵽ����ٶ�ʱ��������.
    unsigned int max_s_lim;
    //���뿪ʼ���ٵ�������(�����û���ٵ�������ٶ�ʱ)��
    unsigned int accel_lim;
	
	a=accel;
	d=decel;

    // �������������������жϷ���
    if(step < 0)//��ʱ��
    {
        //srd.dir = CCW;
		GPIO_ResetBits(GPIOE,GPIO_Pin_5);
        step = -step;
    }
    else//˳ʱ��
    {
        GPIO_SetBits(GPIOE,GPIO_Pin_5); //PE5����� ˳ʱ�뷽��  DRIVER2_DIR
		//srd.dir = CW;
    }
    // ����������
    //DIR(srd.dir);
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
        srd.step_delay = 20;
        // ���õ��Ϊ����״̬
        status.running = TRUE;
        //���ö�ʱ����װֵ	
        TIM_SetAutoreload(TIM8,srd.step_delay);
        //����ռ�ձ�Ϊ50%	
        TIM_SetCompare2(TIM8,srd.step_delay>>1);
        //ʹ�ܶ�ʱ��	      
        TIM_Cmd(TIM8, ENABLE); 
     }
    // ������Ϊ����ƶ�
    else if(step != 0)
    {
   
    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    srd.min_delay = 1000000/speed-1;

    // ��ʼ����Ƶ��Ϊ20Hz
    srd.step_delay = 1000000/20-1;  //f1=20Hz��
		
    // �����������֮��ﵽ����ٶȵ�����
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (((long)(speed-20)/(long)(accel))+1);
		
    // ����ﵽ����ٶ�С��0.5�����壬���ǽ���������Ϊ0
    // ��ʵ�����Ǳ����ƶ�����һ��������ܴﵽ��Ҫ���ٶ�
    if(max_s_lim == 0)
    {
        max_s_lim = 1;
    }

    // �����������֮�����Ǳ��뿪ʼ���٣���Ϊ���м��٣�����������������ɼ����
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = ((long)step*decel) / (accel+decel)-1;
    // ���Ǳ����������1��������ܲ��ܿ�ʼ����.
    if(accel_lim == 0)
    {
        accel_lim = 1;
    }
    // ʹ����������������������õ���������������ʾ��
    //srd.decel_valΪ����
    if(accel_lim <= max_s_lim)//�����ε����
    {
        srd.decel_val = accel_lim - step;
    }
    else//���ε����
    {
        srd.decel_val = -(((long)(speed-20)/(long)(decel))+1);
    }
    // ��ֻʣ��һ�����Ǳ������
    if(srd.decel_val == 0)
    {
        srd.decel_val = -1;
    }

    // ���㿪ʼ����ʱ��������
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
    // ��λ���ٶȼ���ֵ
    srd.decel_count = 0;
    status.running = TRUE;
    //���ö�ʱ����װֵ	
    TIM_SetAutoreload(TIM8,srd.step_delay);
    //����ռ�ձ�Ϊ50%	
    TIM_SetCompare2(TIM8,srd.step_delay>>1);
    //����������
    TIM_SetCounter(TIM8,0);
    //ʹ�ܶ�ʱ��	      
    TIM_Cmd(TIM8, ENABLE); 
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

/******* TIM8�����жϷ������ *********/
//��TIM8->RCR�е��ظ�����ֵ��Ϊ0ʱ�����ж� Ҳ������TIM8->RCR+1����������ж�
void TIM8_UP_TIM13_IRQHandler(void)
{
	 // ������һ����ʱ����
	unsigned int new_step_delay;
    // ���ٹ��������һ����ʱ.
    static int last_accel_delay;
    // �ƶ�������������
    static unsigned int step_count = 0;
	
	if(TIM_GetITStatus(TIM8,TIM_FLAG_Update)!=RESET)//�����ж�
	{
		TIM_ClearITPendingBit(TIM8,TIM_FLAG_Update);//��������жϱ�־λ
		TIM_SetAutoreload(TIM8,srd.step_delay);//�趨�Զ���װֵ	
	    TIM_SetCompare2(TIM8,srd.step_delay>>1);//���ñȽ�ֵ������װ��ֵ��һ�룬��ռ�ձ�Ϊ50%
        TIM_SetCounter(TIM8,0);//����������		
		//�����ֹ����������ֹͣ�˶�
       if(status.out_ena != TRUE)
       {
            srd.run_state = STOP;
       }
	   
	switch(srd.run_state) {
		
    case STOP:
		
      step_count = 0;
      //rest = 0;
      TIM8->CCER &= ~(1<<12); //��ֹ���
      TIM_Cmd(TIM8, DISABLE);
      status.running = FALSE;
      break;

    case ACCEL:
		
	  TIM_GenerateEvent(TIM8,TIM_EventSource_Update);//����һ�������¼� ���³�ʼ��������
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE �����ʹ��
      TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8  
	  //TIM8->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter(srd.dir);
      step_count++;//��¼�Ѿ����͵�������
      srd.accel_count++;//��¼���ٵĴ���
      new_step_delay = 1000000/(20+a*srd.accel_count)-1;
      //rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
      //����ǹ�Ӧ�ÿ�ʼ����
      if(step_count >= srd.decel_start) {
        srd.accel_count = srd.decel_val;  //srd.decel_valΪ���������õ�������   
        srd.run_state = DECEL;
      }
      //����Ƿ񵽴�����������ٶ�
      else if(new_step_delay <= srd.min_delay) {
        last_accel_delay = new_step_delay;  //���ٹ��������һ����ʱ=new_step_delay
        new_step_delay = srd.min_delay;  //�Զ���װ��ֵ����Ϊ����ٶ�
        //rest = 0;
        srd.run_state = RUN;
	
      }
      break;

    case RUN:
		
	  TIM_GenerateEvent(TIM8,TIM_EventSource_Update);//����һ�������¼� ���³�ʼ��������
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE �����ʹ��
      TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8
	  //TIM8->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter(srd.dir);
      step_count++;
      new_step_delay = srd.min_delay;
      //����Ƿ���Ҫ��ʼ����
      if(step_count >= srd.decel_start) {
        srd.accel_count = srd.decel_val;//�Ѽ��ٵ������������ټ�������������
        new_step_delay = srd.min_delay;
		//�����һ�μ��ٵ���ʱ��Ϊ��ʼ���ٵ���ʱ
        //new_step_delay = last_accel_delay;
        srd.run_state = DECEL;
		 
      }
      break;

    case DECEL:
		
	  TIM_GenerateEvent(TIM8,TIM_EventSource_Update);//����һ�������¼� ���³�ʼ��������
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE �����ʹ��
      TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8
	  //TIM8->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter(srd.dir);
      step_count++;
      srd.accel_count++;
	  srd.decel_count++;
	  new_step_delay = 1000000/((1000000/(srd.min_delay+1))-d*srd.decel_count)-1;
	
     
	 if(new_step_delay>=49999)
	 {
	   new_step_delay=49999;
     }
      if(srd.accel_count >= 0){    //��srd.accel_countС��0˵����û������
        srd.run_state = STOP;
		  a=0;
		  d=0;
      }
      break;
      }
		
	srd.step_delay = new_step_delay;
	
	}	
}
	


