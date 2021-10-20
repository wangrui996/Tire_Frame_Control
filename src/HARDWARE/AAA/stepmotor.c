#include "stepmotor.h"
#if STEP_MOTOR

StepMotorAddSpeed MotorAddSpeed;




/*******************************************************************************
*							定时器4初始化函数
*
*函数名：void TIM5_Int_Init(u16 arr,u16 psc)
*
*形参：	arr 		预装载值
		psc			分频系数
*
*返回值：	无
*
*
*******************************************************************************/
void TIM4_Int_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟 
 
   //设置该引脚为复用输出功能,输出TIM4 CH3的PWM脉冲波形	GPIOB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 			//TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//初始化GPIO
 
   //初始化TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; 						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  						//根据T指定的参数初始化外设TIM3 OC2
 
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 				 //使能TIM3在CCR2上的预装载寄存器
 
//	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}

u32 PluseNum = 0;

/*******************************************************************************
*							步进电机初始化
*
*函数名：void MotorInit(void)
*
*形参：	无
*
*返回值：	无
*
*
*******************************************************************************/
void MotorInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	GPIO_ResetBits(GPIOE,GPIO_Pin_14|GPIO_Pin_15);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);	

	MF = 0;
	DIR = 1;
	DE = 0;
	
	TIM3_Int_Init(1000-1);	//计算PWM波的脉冲来进入中断
	TIM4_Int_Init(1000,71);	//1Mhz.1ms一个10us的脉冲 步进电机的脉冲必须大于2.5us
	TIM_SetCompare3(TIM4,10);
	delay_ms(10);
}

/*******************************************************************************
*							步进电机运行
*
*函数名：void MotorStart(u8 UpORDown,u8 speed,u16 tim3_cnt_int)
*
*形参：	UpORDown	升降台运行方向
		speed		升降台运行速度 1000即1ms一个脉冲 脉冲越快电机运行越快
		tim3_cnt_int	TIM3写入的溢出值
*
*返回值：	无
*
*
*******************************************************************************/
void MotorStart(u8 UpORDown,u16 speed,u16 tim3_cnt_int)
{
	TIM4->ARR = speed-1;
	TIM3->ARR = tim3_cnt_int-1;
	
	MF = 1;	//电机使能信号
	DR = UpORDown;		//电机方向

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE); 
}



/*******************************************************************************
*							步进电机停止
*
*函数名：void MotorStart(u8 UpORDown,u8 speed)
*
*形参：	void
*
*返回值：void
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
*							步进电机运行给定脉冲
*
*函数名：void MotorStartPluse(u32 pluse,u8 UpORDown)
*
*形参：	pluse		脉冲数
		UpORDown	升降台方向
*
*返回值：void
*
*描述：	步进电机匀加速时间和加速次数，起始频率和目标频率可以再宏定义中的修改
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
	else if(UpORDown==DOWN && (MotorAddSpeed.flag & (1<<7))==1)	return;//在最低位置不能下降

	if(pluse>=NEED_PLUSE)												//适合匀加速运动
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
				MotorAddSpeed.Add_Minus==ADD)								//达到加速条件 在加速阶段											
			{
//				u2_printf("1111111\t%d\r\n",MotorAddSpeed.pluse_send);//调试串口输出
				MotorAddSpeed.Add_Cnt++;
				Next_F 		 =	START_F+(MotorAddSpeed.Add_Cnt*ADD_SPEED);
				MotorAddSpeed.Arr_tim4_start	=	TIMER_F/Next_F;
				add_pluse 	+= T_ADD*Next_F/1000;							//计算出下次需要加速时的累计脉冲值
				TIM4->ARR	 = MotorAddSpeed.Arr_tim4_start-1;	
			}
			
			else if(MotorAddSpeed.pluse_send >= pluse-(NEED_PLUSE/2) && Next_F>START_F && MotorAddSpeed.pluse_send>=add_pluse\
					&& MotorAddSpeed.Add_Minus==MINUS && MotorAddSpeed.Add_Cnt>0		 )						//达到减速条件 减速阶段	 			
			{
//				u2_printf("2222222\t%d\r\n",MotorAddSpeed.pluse_send);//调试串口输出
				MotorAddSpeed.Add_Cnt--;
				Next_F 		 =	START_F+(MotorAddSpeed.Add_Cnt*ADD_SPEED);
				MotorAddSpeed.Arr_tim4_start	=	TIMER_F/Next_F;
				add_pluse 	+= T_ADD*Next_F/1000;							//计算出下次需要减速时的累计脉冲值
				TIM4->ARR	 = MotorAddSpeed.Arr_tim4_start-1;					
			}
			
			else if(Next_F==OBJ_F && MotorAddSpeed.pluse_send < pluse-(NEED_PLUSE/2))	//匀速阶段
			{}			

			if(MotorAddSpeed.pluse_send>=pluse)							//脉冲计数已到达
			{
				MotorStop();
				if(UpORDown==UP)
					MotorAddSpeed.pluse_Current +=MotorAddSpeed.pluse_send;
				else if(UpORDown==DOWN)
					MotorAddSpeed.pluse_Current -=MotorAddSpeed.pluse_send;
//				u2_printf("3333333\t%d\r\n",MotorAddSpeed.pluse_Current);//调试串口输出
				break;
			}
			if(MotorAddSpeed.flag & (1<<7))	break;
			else		MotorAddSpeed.flag &= ~(1<<7);	
		}	
	}
	
	else if(pluse>=1000 && pluse < NEED_PLUSE)							//脉冲太少不适合匀加速
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
				
//				u2_printf("%d\r\n",MotorAddSpeed.pluse_Current);//调试串口输出
				break;
			}
			if(MotorAddSpeed.flag	&	(1<<7))	break;
			else	MotorAddSpeed.flag &= ~(1<<7);
		}
	}
	else if(pluse<1000)															//脉冲太少适合低速运动
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
//				u2_printf("%d\r\n",MotorAddSpeed.pluse_Current);		//调试串口输出
				break;
			}	
			if(MotorAddSpeed.flag	&	(1<<7))	break;		
			else	MotorAddSpeed.flag &= ~(1<<7);
		}
	}
}
/*******************************************************************************
*							定时器3初始化函数
*
*函数名：void TIM5_Int_Init(u16 arr,u16 psc)
*
*形参：	arr 		预装载值
*
*返回值：	无
*
*描述：	定时器3使用从模式下的外部时钟源2，PWM的脉冲也输出到TIM3_ETR引脚PD2该引脚配置为浮空输入
*
*
例如，要配置在ETR下每2个上升沿计数一次的向上计数器，使用下列步骤：
1. 本例中不需要滤波器，置TIMx_SMCR寄存器中的ETF[3:0]=0000 
2. 设置预分频器，置TIMx_SMCR寄存器中的ETPS[1:0]=01 
3. 设置在ETR的上升沿检测，置TIMx_SMCR寄存器中的ETP=0 
4. 开启外部时钟模式2，置TIMx_SMCR寄存器中的ECE=1 
5. 启动计数器，置TIMx_CR1寄存器中的CEN=1
*******************************************************************************/
void TIM3_Int_Init(u16 arr)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能GPIO外设
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC->APB1ENR |= 1<<1;		//使能TIM3时钟
	TIM3->ARR=arr;   			//设定计数器自动重装值
	TIM3->PSC=0;        		//不分频
	TIM3->SMCR &= ~(0xf<<8);	//无滤波
	TIM3->SMCR &= ~(3<<12);		//关闭预分频
	TIM3->SMCR &= ~(1<<15);		//ETR  1低电平或下降沿有效   0高电平或上升沿有效
	TIM3->SMCR |= 1<<14;		//使能外部时钟模式2
	TIM3->DIER |= 1<<0;			//允许更新中断
	TIM3->DIER |= 1<<6;			//允许触发中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  			//TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  		//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  							//初始化NVIC寄存器

	TIM3->CNT = 0x00;	//清零计数器
	TIM3->CR1 |= 1<<0;	//使能定时器，开启计数。				 
}

/*******************************************************************************
*							定时器3中断服务函数
*
*函数名：void TIM5_Int_Init(u16 arr,u16 psc)
*
*形参：	arr 		预装载值
*
*返回值：	无
*
*
*******************************************************************************/
void TIM3_IRQHandler(void)   //TIM3中断 
{	
	if(TIM3->SR&0X0001)//溢出中断
	{
		TIM3->SR&=~(1<<0);//清除中断标志位 
		MotorAddSpeed.pluse_send +=2;	
	}   
}
#endif



