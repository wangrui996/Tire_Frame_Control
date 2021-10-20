#include "motor1.h"
#include "sys.h"
#include "math.h"
#include "rs485.h"
//˿�ܲ��������������	
//�޸�����:2020/03/29
//�汾��V2.0

/**********˿�������� �˿ڶ��� **************
//DRIVER1_DIR   PE3   ˿�ܷ���
//DRIVER1_OE    PE4   ˿��������ʹ��
//STEP_PULSE1   PE11(TIM1_CH2)  ˿������������
******************************************/

//ϵͳ�Ӽ��ٲ���
speedRampData1 srd1;
//��¼���������λ��
int stepPosition1 = 0;
//ϵͳ���������״̬
struct GLOBAL_FLAGS1 status1 = {FALSE, FALSE,TRUE};
unsigned static int a1=0;
unsigned static int d1=0;

void Driver1_Init(void)  //�����������ź��߳�ʼ��  ʹ�� ˳ʱ�뷽��
{
	/***** ˿�������������ź��߳�ʼ��*******/     //PE3 ����  PE4 ʹ��
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //ʹ��GPIOEʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4;//DRIVER2_DIR DRIVER2_OE��Ӧ����PE5 PE6
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;       //��ͨ���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      //�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOE, &GPIO_InitStructure);  //��ʼ��GPIOE
	
	
	GPIO_SetBits(GPIOE,GPIO_Pin_3); //PE3����� ˳ʱ�뷽��  DRIVER1_DIR
	GPIO_ResetBits(GPIOE,GPIO_Pin_4); //PE4����� ʹ�����  DRIVER1_OE
	
}
/***********************************************
//TIM1_CH2(PE11) ���������+�ظ��������ܳ�ʼ�� 
//TIM1 ʱ��Ƶ�� 84*2=168MHz
//arr:�Զ���װֵ
//psc:ʱ��Ԥ��Ƶ��
************************************************/
void TIM1_OPM_RCR_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��	
	
	/* GPIOE11����Ϊ��ʱ��1  */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;           //TIM1_ch2��ӦIO��GPIOE11
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;      //����             Ϊʲô��������
	GPIO_Init(GPIOE, &GPIO_InitStructure);  //��ʼ��GPIOE11
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOE11����Ϊ��ʱ��1
	
	/* ��ʼ����ʱ��1  */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ ��ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��psc+1��
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //����ʱ�ӷָ� TIM_CKD_DIV1=0
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //��ʼ��TIM1	
	
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);//�����ʱ��TIM1���ж�TIM_IT��־λ
	TIM_UpdateRequestConfig(TIM1,TIM_UpdateSource_Regular); /********* ����ֻ�м��������Ϊ�����ж� ********/
	TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);/******* ������ģʽ **********/
	
	/*��ʼ��TIM1��ͨ��2*/
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�,,PWM������ʼ��ƽΪ��
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//�Ƚ����2ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; /****** �Ƚ����2Nʧ��????? *******/
	TIM_OCInitStructure.TIM_Pulse = arr>>1; // PWM�ıȽ�ֵ�����ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIM1��ͨ��2
	
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //TIM1��CH2Ԥװ��ʹ��
	TIM_ARRPreloadConfig(TIM1, ENABLE); //ʹ��TIM1��ARR�ϵ�Ԥװ�ؼĴ���
	
	/*��ʼ��TIM1�ж�*/
	TIM_ITConfig(TIM1, TIM_IT_Update ,ENABLE);  //TIM1ʹ�ܻ���ʧ��ָ����TIM�жϺ���
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn;  //TIM1�����ж�;  NVIC_IRQChannelȷ������һ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;//��ռ���ȼ�1��  NVIC_IRQChannelPreemptionPriority��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;// ��Ӧ���ȼ�1��        NVIC_IRQChannelSubPriority��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //���TIM1���жϴ�����λ:TIM �ж�Դ
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	
	
}	

void MSD_ENA1(FunctionalState NewState)
{
    if(NewState)
    {
      //ENAʧ�ܣ���ֹ���������
      GPIO_SetBits(GPIOE,GPIO_Pin_4);
      //����ֹͣ��־λΪ��
      status1.out_ena = FALSE; 
      //printf("\n\r��������ֹ������ѻ�״̬����ʱ���Ϊ�ޱ�������״̬�������ֶ���ת���");        
    }
    else
    {
      //ENAʹ��
      GPIO_ResetBits(GPIOE,GPIO_Pin_4);
      //����ֹͣ��־λΪ��
      status1.out_ena = TRUE; 
      //printf("\n\r�������ָ����У���ʱ���Ϊ��������״̬����ʱ����ָ������������Ƶ��");         
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
void MSD_Move1(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
	//�ﵽ����ٶ�ʱ��������.
    unsigned int max_s_lim;
    //���뿪ʼ���ٵ�������(�����û���ٵ�������ٶ�ʱ)��
    unsigned int accel_lim;
	
	a1=accel;
	d1=decel;

    // �������������������жϷ���
    if(step < 0)//��ʱ��
    {
        //srd.dir = CCW;
		GPIO_ResetBits(GPIOE,GPIO_Pin_3);
        step = -step;
    }
    else//˳ʱ��
    {
        GPIO_SetBits(GPIOE,GPIO_Pin_3); //PE3����� ˳ʱ�뷽��  DRIVER2_DIR
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
        srd1.accel_count = -1;
        // ����״̬
        srd1.run_state = DECEL;
        // ����ʱ
        srd1.step_delay = 20;
        // ���õ��Ϊ����״̬
        status1.running = TRUE;
        //���ö�ʱ����װֵ	
        TIM_SetAutoreload(TIM1,srd1.step_delay);
        //����ռ�ձ�Ϊ50%	
        TIM_SetCompare2(TIM1,srd1.step_delay>>1);
        //ʹ�ܶ�ʱ��	      
        TIM_Cmd(TIM1, ENABLE); 
     }
    // ������Ϊ����ƶ�
    else if(step != 0)
    {
   
    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    srd1.min_delay = 1000000/speed-1;

    srd1.step_delay = 1000000/20-1;  //f1=20Hz��
		
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
        srd1.decel_val = accel_lim - step;
    }
    else//���ε����
    {
        srd1.decel_val = -(((long)(speed-20)/(long)(decel))+1);
    }
    // ��ֻʣ��һ�����Ǳ������
    if(srd1.decel_val == 0)
    {
        srd1.decel_val = -1;
    }

    // ���㿪ʼ����ʱ��������
    srd1.decel_start = step + srd1.decel_val;
	
    // �������ٶȺ��������ǾͲ���Ҫ���м����˶�
    if(srd1.step_delay <= srd1.min_delay)
    {
        srd1.step_delay = srd1.min_delay;
        srd1.run_state = RUN;
    }
    else
   {
        srd1.run_state = ACCEL;
   }
   
    // ��λ���ٶȼ���ֵ
    srd1.accel_count = 0;
    // ��λ���ٶȼ���ֵ
    srd1.decel_count = 0;
    status1.running = TRUE;
    //���ö�ʱ����װֵ	
    TIM_SetAutoreload(TIM1,srd1.step_delay);
    //����ռ�ձ�Ϊ50%	
    TIM_SetCompare2(TIM1,srd1.step_delay>>1);
    //����������
    TIM_SetCounter(TIM1,0);
    //ʹ�ܶ�ʱ��	      
    TIM_Cmd(TIM1, ENABLE); 
    }
}

void MSD_StepCounter1(signed char inc)
{
  //���ݷ����жϵ��λ��
  if(inc == CCW1)
  {
    stepPosition1--;
  }
  else
  {
    stepPosition1++;
  }
}


/******* TIM1�����жϷ������ *********/
//��TIM1->RCR�е��ظ�����ֵ��Ϊ0ʱ�����ж� Ҳ������TIM8->RCR+1����������ж�
void TIM1_UP_TIM10_IRQHandler(void)
{
  
	// ������һ����ʱ����
	unsigned int new_step_delay;
    // ���ٹ��������һ����ʱ.
    static int last_accel_delay;
    // �ƶ�������������
    static unsigned int step_count = 0;
	
	
	if(TIM_GetITStatus(TIM1,TIM_FLAG_Update)!=RESET)//�����ж�
	{
		TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);//��������жϱ�־λ
		TIM_SetAutoreload(TIM1,srd1.step_delay);//�趨�Զ���װֵ	
	    TIM_SetCompare2(TIM1,srd1.step_delay>>1);//���ñȽ�ֵ������װ��ֵ��һ�룬��ռ�ձ�Ϊ50%
        TIM_SetCounter(TIM1,0);//����������		
		//�����ֹ����������ֹͣ�˶�
       if(status1.out_ena != TRUE)
       {
            srd1.run_state = STOP;
       }
	switch(srd1.run_state) {
    case STOP:
      step_count = 0;
      //rest = 0;
      TIM1->CCER &= ~(1<<12); //��ֹ���
      TIM_Cmd(TIM1, DISABLE);
      status1.running = FALSE;
      break;

    case ACCEL:
	  TIM_GenerateEvent(TIM1,TIM_EventSource_Update);//����һ�������¼� ���³�ʼ��������
	  TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��
      TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1  
	  //TIM8->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter1(srd1.dir);
      step_count++;//��¼�Ѿ����͵�������
      srd1.accel_count++;//��¼���ٵĴ���
      new_step_delay = 1000000/(20+a1*srd1.accel_count)-1;
      //rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
      //����ǹ�Ӧ�ÿ�ʼ����
      if(step_count >= srd1.decel_start) {
        srd1.accel_count = srd1.decel_val;  //srd.decel_valΪ���������õ�������   
        srd1.run_state = DECEL;
      }
      //����Ƿ񵽴�����������ٶ�
      else if(new_step_delay <= srd1.min_delay) 
	  {
        last_accel_delay = new_step_delay;  //���ٹ��������һ����ʱ=new_step_delay
        new_step_delay = srd1.min_delay;  //�Զ���װ��ֵ����Ϊ����ٶ�
        //rest = 0;
        srd1.run_state = RUN;

      }
      break;

    case RUN:
	  TIM_GenerateEvent(TIM1,TIM_EventSource_Update);//����һ�������¼� ���³�ʼ��������
	  TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��
      TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	  //TIM8->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter1(srd1.dir);
      step_count++;
      new_step_delay = srd1.min_delay;
      //����Ƿ���Ҫ��ʼ����
      if(step_count >= srd1.decel_start) {
        srd1.accel_count = srd1.decel_val;//�Ѽ��ٵ������������ټ�������������
        new_step_delay = srd1.min_delay;
		//�����һ�μ��ٵ���ʱ��Ϊ��ʼ���ٵ���ʱ
        //new_step_delay = last_accel_delay;
        srd1.run_state = DECEL;
		  
      }
      break;

    case DECEL:
	  
	  TIM_GenerateEvent(TIM1,TIM_EventSource_Update);//����һ�������¼� ���³�ʼ��������
	  TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��
      TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	  //TIM8->CCER |= 1<<12; //ʹ�����
      MSD_StepCounter1(srd1.dir);
      step_count++;
      srd1.accel_count++;
	  srd1.decel_count++;
	  new_step_delay = 1000000/((1000000/(srd1.min_delay+1))-d1*srd1.decel_count)-1;
	
     
      //����Ƿ�Ϊ���һ��
	 if(new_step_delay>=49999)
	 {
	   new_step_delay=49999;
     }
      if(srd1.accel_count >= 0){    //��srd.accel_countС��0˵����û������
        srd1.run_state = STOP;
	

		  a1=0;
		  d1=0;
      }
      break;
      }	
	srd1.step_delay = new_step_delay;
	}	
}
