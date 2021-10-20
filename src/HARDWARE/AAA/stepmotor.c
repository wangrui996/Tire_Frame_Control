#include "stepmotor.h"
#if STEP_MOTOR

StepMotorAddSpeed MotorAddSpeed;




/*******************************************************************************
*							��ʱ��4��ʼ������
*
*��������void TIM5_Int_Init(u16 arr,u16 psc)
*
*�βΣ�	arr 		Ԥװ��ֵ
		psc			��Ƶϵ��
*
*����ֵ��	��
*
*
*******************************************************************************/
void TIM4_Int_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ�� 
 
   //���ø�����Ϊ�����������,���TIM4 CH3��PWM���岨��	GPIOB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 			//TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//��ʼ��GPIO
 
   //��ʼ��TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; 						//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������:TIM����Ƚϼ��Ը�
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  						//����Tָ���Ĳ�����ʼ������TIM3 OC2
 
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 				 //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
 
//	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
}

u32 PluseNum = 0;

/*******************************************************************************
*							���������ʼ��
*
*��������void MotorInit(void)
*
*�βΣ�	��
*
*����ֵ��	��
*
*
*******************************************************************************/
void MotorInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	GPIO_ResetBits(GPIOE,GPIO_Pin_14|GPIO_Pin_15);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);	

	MF = 0;
	DIR = 1;
	DE = 0;
	
	TIM3_Int_Init(1000-1);	//����PWM���������������ж�
	TIM4_Int_Init(1000,71);	//1Mhz.1msһ��10us������ �������������������2.5us
	TIM_SetCompare3(TIM4,10);
	delay_ms(10);
}

/*******************************************************************************
*							�����������
*
*��������void MotorStart(u8 UpORDown,u8 speed,u16 tim3_cnt_int)
*
*�βΣ�	UpORDown	����̨���з���
		speed		����̨�����ٶ� 1000��1msһ������ ����Խ��������Խ��
		tim3_cnt_int	TIM3д������ֵ
*
*����ֵ��	��
*
*
*******************************************************************************/
void MotorStart(u8 UpORDown,u16 speed,u16 tim3_cnt_int)
{
	TIM4->ARR = speed-1;
	TIM3->ARR = tim3_cnt_int-1;
	
	MF = 1;	//���ʹ���ź�
	DR = UpORDown;		//�������

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE); 
}



/*******************************************************************************
*							�������ֹͣ
*
*��������void MotorStart(u8 UpORDown,u8 speed)
*
*�βΣ�	void
*
*����ֵ��void
*
*
*******************************************************************************/
void MotorStop(void)
{	
	TIM_Cmd(TIM4, DISABLE); 
	TIM_Cmd(TIM3, DISABLE); 	
//	MF = 0;
//	DR = 1;	
}

/*******************************************************************************
*							����������и�������
*
*��������void MotorStartPluse(u32 pluse,u8 UpORDown)
*
*�βΣ�	pluse		������
		UpORDown	����̨����
*
*����ֵ��void
*
*������	��������ȼ���ʱ��ͼ��ٴ�������ʼƵ�ʺ�Ŀ��Ƶ�ʿ����ٺ궨���е��޸�
*
*
*******************************************************************************/
void MotorStartPluse(u32 pluse,u8 UpORDown)
{
	u16 add_pluse = 0;
	u16	Next_F = 0;
	u8 a = 1;
	delay_ms(10);
	MotorAddSpeed.pluse_send = 0;
	
	if(UpORDown==UP)
	{
		MotorAddSpeed.flag				|=	1<<1;
		MotorAddSpeed.flag				&=	 ~(1<<7);
		MotorAddSpeed.pluse_ObjSend 	= 	pluse;
	}
	else if(UpORDown==DOWN && (MotorAddSpeed.flag & (1<<7))!=1)
	{
		MotorAddSpeed.flag				&=	~(1<<1);
		MotorAddSpeed.pluse_ObjSend 	=	pluse;	
	}
	else if(UpORDown==DOWN && (MotorAddSpeed.flag & (1<<7))==1)	return;//�����λ�ò����½�

	if(pluse>=NEED_PLUSE)												//�ʺ��ȼ����˶�
	{
		MotorAddSpeed.Arr_tim4_start	=	TIMER_F/START_F;
		MotorAddSpeed.Add_Cnt			=	0;
		MotorAddSpeed.Add_Minus			=	ADD;
		MotorAddSpeed.Arr_tim3			=	2;
		Next_F 		 =	START_F+(MotorAddSpeed.Add_Cnt*ADD_SPEED);
		add_pluse = T_ADD*Next_F/1000;

		MotorStart(UpORDown,MotorAddSpeed.Arr_tim4_start,MotorAddSpeed.Arr_tim3);
		while(1)
		{
			if( MotorAddSpeed.pluse_send >= pluse-(NEED_PLUSE/2) && a==1)
			{
				a=0;
				MotorAddSpeed.Add_Minus			=	MINUS;
				add_pluse = MotorAddSpeed.pluse_send;
			}
			
			if(MotorAddSpeed.pluse_send>=add_pluse && Next_F<OBJ_F && \
				MotorAddSpeed.Add_Minus==ADD)								//�ﵽ�������� �ڼ��ٽ׶�											
			{
//				u2_printf("1111111\t%d\r\n",MotorAddSpeed.pluse_send);//���Դ������
				MotorAddSpeed.Add_Cnt++;
				Next_F 		 =	START_F+(MotorAddSpeed.Add_Cnt*ADD_SPEED);
				MotorAddSpeed.Arr_tim4_start	=	TIMER_F/Next_F;
				add_pluse 	+= T_ADD*Next_F/1000;							//������´���Ҫ����ʱ���ۼ�����ֵ
				TIM4->ARR	 = MotorAddSpeed.Arr_tim4_start-1;	
			}
			
			else if(MotorAddSpeed.pluse_send >= pluse-(NEED_PLUSE/2) && Next_F>START_F && MotorAddSpeed.pluse_send>=add_pluse\
					&& MotorAddSpeed.Add_Minus==MINUS && MotorAddSpeed.Add_Cnt>0		 )						//�ﵽ�������� ���ٽ׶�	 			
			{
//				u2_printf("2222222\t%d\r\n",MotorAddSpeed.pluse_send);//���Դ������
				MotorAddSpeed.Add_Cnt--;
				Next_F 		 =	START_F+(MotorAddSpeed.Add_Cnt*ADD_SPEED);
				MotorAddSpeed.Arr_tim4_start	=	TIMER_F/Next_F;
				add_pluse 	+= T_ADD*Next_F/1000;							//������´���Ҫ����ʱ���ۼ�����ֵ
				TIM4->ARR	 = MotorAddSpeed.Arr_tim4_start-1;					
			}
			
			else if(Next_F==OBJ_F && MotorAddSpeed.pluse_send < pluse-(NEED_PLUSE/2))	//���ٽ׶�
			{}			

			if(MotorAddSpeed.pluse_send>=pluse)							//��������ѵ���
			{
				MotorStop();
				if(UpORDown==UP)
					MotorAddSpeed.pluse_Current +=MotorAddSpeed.pluse_send;
				else if(UpORDown==DOWN)
					MotorAddSpeed.pluse_Current -=MotorAddSpeed.pluse_send;
//				u2_printf("3333333\t%d\r\n",MotorAddSpeed.pluse_Current);//���Դ������
				break;
			}
			if(MotorAddSpeed.flag & (1<<7))	break;
			else		MotorAddSpeed.flag &= ~(1<<7);	
		}	
	}
	
	else if(pluse>=1000 && pluse < NEED_PLUSE)							//����̫�ٲ��ʺ��ȼ���
	{
		MotorAddSpeed.flag				&=	~(1<<0);
		MotorAddSpeed.Finish			=	0;
		MotorStart(UpORDown,LOWEST_SPEED,2);
		while(1)
		{
			if(MotorAddSpeed.pluse_send>=200 && MotorAddSpeed.Finish==0)
			{
				MotorStart(UpORDown,LOWEST_SPEED-5000-2500,2);
				MotorAddSpeed.Finish = 1;
			}
			if(MotorAddSpeed.pluse_send>=(pluse-200) && MotorAddSpeed.Finish==1)
			{
				MotorStart(UpORDown,LOWEST_SPEED,2);
				MotorAddSpeed.Finish = 2;
			}
			if(MotorAddSpeed.pluse_send>=pluse && MotorAddSpeed.Finish == 2)
			{
				MotorStop();

				if(UpORDown == UP)
					MotorAddSpeed.pluse_Current += MotorAddSpeed.pluse_send;
				else if(UpORDown == DOWN)
					MotorAddSpeed.pluse_Current -= MotorAddSpeed.pluse_send;	
				
//				u2_printf("%d\r\n",MotorAddSpeed.pluse_Current);//���Դ������
				break;
			}
			if(MotorAddSpeed.flag	&	(1<<7))	break;
			else	MotorAddSpeed.flag &= ~(1<<7);
		}
	}
	else if(pluse<1000)															//����̫���ʺϵ����˶�
	{
		MotorAddSpeed.flag		&=	~(1<<0);
		MotorStart(UpORDown,LOWEST_SPEED-5000,2);
		while(1)
		{
			if(MotorAddSpeed.pluse_send>=pluse)							
			{
				MotorStop();	
				if(UpORDown == UP)
					MotorAddSpeed.pluse_Current += MotorAddSpeed.pluse_send;
				else if(UpORDown == DOWN)
					MotorAddSpeed.pluse_Current -= MotorAddSpeed.pluse_send;					
//				u2_printf("%d\r\n",MotorAddSpeed.pluse_Current);		//���Դ������
				break;
			}	
			if(MotorAddSpeed.flag	&	(1<<7))	break;		
			else	MotorAddSpeed.flag &= ~(1<<7);
		}
	}
}
/*******************************************************************************
*							��ʱ��3��ʼ������
*
*��������void TIM5_Int_Init(u16 arr,u16 psc)
*
*�βΣ�	arr 		Ԥװ��ֵ
*
*����ֵ��	��
*
*������	��ʱ��3ʹ�ô�ģʽ�µ��ⲿʱ��Դ2��PWM������Ҳ�����TIM3_ETR����PD2����������Ϊ��������
*
*
���磬Ҫ������ETR��ÿ2�������ؼ���һ�ε����ϼ�������ʹ�����в��裺
1. �����в���Ҫ�˲�������TIMx_SMCR�Ĵ����е�ETF[3:0]=0000 
2. ����Ԥ��Ƶ������TIMx_SMCR�Ĵ����е�ETPS[1:0]=01 
3. ������ETR�������ؼ�⣬��TIMx_SMCR�Ĵ����е�ETP=0 
4. �����ⲿʱ��ģʽ2����TIMx_SMCR�Ĵ����е�ECE=1 
5. ��������������TIMx_CR1�Ĵ����е�CEN=1
*******************************************************************************/
void TIM3_Int_Init(u16 arr)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ��GPIO����
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC->APB1ENR |= 1<<1;		//ʹ��TIM3ʱ��
	TIM3->ARR=arr;   			//�趨�������Զ���װֵ
	TIM3->PSC=0;        		//����Ƶ
	TIM3->SMCR &= ~(0xf<<8);	//���˲�
	TIM3->SMCR &= ~(3<<12);		//�ر�Ԥ��Ƶ
	TIM3->SMCR &= ~(1<<15);		//ETR  1�͵�ƽ���½�����Ч   0�ߵ�ƽ����������Ч
	TIM3->SMCR |= 1<<14;		//ʹ���ⲿʱ��ģʽ2
	TIM3->DIER |= 1<<0;			//��������ж�
	TIM3->DIER |= 1<<6;			//�������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  			//TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  		//�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//��ʼ��NVIC�Ĵ���

	TIM3->CNT = 0x00;	//���������
	TIM3->CR1 |= 1<<0;	//ʹ�ܶ�ʱ��������������				 
}

/*******************************************************************************
*							��ʱ��3�жϷ�����
*
*��������void TIM5_Int_Init(u16 arr,u16 psc)
*
*�βΣ�	arr 		Ԥװ��ֵ
*
*����ֵ��	��
*
*
*******************************************************************************/
void TIM3_IRQHandler(void)   //TIM3�ж� 
{	
	if(TIM3->SR&0X0001)//����ж�
	{
		TIM3->SR&=~(1<<0);//����жϱ�־λ 
		MotorAddSpeed.pluse_send +=2;	
	}   
}
#endif



