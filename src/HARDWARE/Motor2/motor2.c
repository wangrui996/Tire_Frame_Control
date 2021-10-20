#include "motor2.h"
#include "usart.h"
#include "sys.h"
#include "math.h"
#include "rs485.h"

//底盘步进电机驱动器 测试代码	
//修改日期:2020/05/20
//版本：V2.0

/**********底盘驱动器 端口定义 **************
//DRIVER1_DIR   PE5 (DCMI_D6)   底盘方向
//DRIVER1_OE    PE6 (DCMI_D7)   底盘驱动器使能
//STEP_PULSE1   PC7 (TIM8_CH2  DCMI_D1)
******************************************/
u8 rcr_remainder;   //重复计数余数部分  rcr_remainder为0-255
long rcr_integer;	//重复计数整数部分
u8 is_rcr_finish=1; //脉冲发送是否完成   完成为1
long target_pos=0;  //目标位置-（目标脉冲数）
long current_pos=0; //当前位置-（当前脉冲数）
//DIR_Type motor_dir=CW;//CW即为1，1顺时针

//系统加减速参数
speedRampData srd;
//记录步进电机的位置
int stepPosition = 0;
//系统电机、串口状态
struct GLOBAL_FLAGS status = {FALSE, FALSE,TRUE};
unsigned static int a=0;
unsigned static int d=0;

/************** 驱动器控制信号线初始化 ****************/
void Driver_Init(void)
{
     /***** 底盘驱动器控制信号线初始化*******/     //PE5 方向  PE6 使能
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //使能GPIOE时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;//DRIVER2_DIR DRIVER2_OE对应引脚PE5 PE6
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;       //普通输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);  //初始化GPIOE5
	
	GPIO_SetBits(GPIOE,GPIO_Pin_5); //PE5输出高 顺时针方向  DRIVER2_DIR
	GPIO_ResetBits(GPIOE,GPIO_Pin_6); //PE6输出低 使能输出  DRIVER2_OE
	
}	
/***********************************************
//TIM8_CH2(PC7) 单脉冲输出+重复计数功能初始化
//TIM8 时钟频率 84*2=168MHz
//arr:自动重装值
//psc:时钟预分频数
************************************************/
void TIM8_OPM_RCR_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8时钟使能  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟	
	
	/* GPIOC7复用为定时器8  */
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;           //TIM8_ch2对应IO口GPIOC7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;      //下拉             为什么是下拉？
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化GPIOC7
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); //GPIOC7复用为定时器8
	
	/* 初始化定时器8  */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc; //预分频系数：设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //设置时钟分割 TIM_CKD_DIV1=0
	
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //初始化TIM8
	
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);//清除定时器TIM8的中断TIM_IT标志位
	TIM_UpdateRequestConfig(TIM8,TIM_UpdateSource_Regular); /********* 设置只有计数溢出作为更新中断 ********/
	TIM_SelectOnePulseMode(TIM8,TIM_OPMode_Single);/******* 单脉冲模式 **********/
	
	/*初始化TIM8的通道2*/
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高,,PWM波的起始电平为高
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//比较输出2使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; /****** 比较输出2N失能????? *******/
	TIM_OCInitStructure.TIM_Pulse = arr>>1; // PWM的比较值：设置待装入捕获比较寄存器的脉冲值
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIM8的通道2
	
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //TIM8的CH2预装载使能	 
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIM8在ARR上的预装载寄存器
	
	
	/*初始化TIM8中断*/
	TIM_ITConfig(TIM8, TIM_IT_Update ,ENABLE);  //TIM8使能或者失能指定的TIM中断函数
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn;  //TIM8更新中断;  NVIC_IRQChannel确定是哪一个中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;//抢占优先级1级  NVIC_IRQChannelPreemptionPriority抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;// 响应优先级1级        NVIC_IRQChannelSubPriority响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
}


void MSD_ENA(FunctionalState NewState)
{
    if(NewState)
    {
      //ENA失能，禁止驱动器输出
      GPIO_SetBits(GPIOE,GPIO_Pin_6);
      //紧急停止标志位为真
      status.out_ena = FALSE; 
      printf("\n\r驱动器禁止输出（脱机状态）此时电机为无保持力矩状态，可以手动旋转电机");        
    }
    else
    {
      //ENA使能
      GPIO_ResetBits(GPIOE,GPIO_Pin_6);
      //紧急停止标志位为假
      status.out_ena = TRUE; 
      printf("\n\r驱动器恢复运行，此时电机为保持力矩状态，此时串口指令可以正常控制电机");         
    }
    
}

/*! \brief 以给定的脉冲数移动步进电机
 *  通过计算加速到最大速度，以给定的脉冲数开始减速
 *  如果加速度和减速度很小，步进电机会移动很慢，还没达到最大速度就要开始减速
 *  \param step   移动的脉冲数 (正数为顺时针，负数为逆时针).
 *  \param accel  加速度,如果取值为10，10Hz/脉冲
 *  \param decel  减速度,如果取值为10，10Hz/脉冲
 *  \param speed  最大频率,如果取值为100,最大频率100Hz
 */
void MSD_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
	//达到最大速度时的脉冲数.
    unsigned int max_s_lim;
    //必须开始减速的脉冲数(如果还没加速到达最大速度时)。
    unsigned int accel_lim;
	
	a=accel;
	d=decel;

    // 根据脉冲数的正负来判断方向
    if(step < 0)//逆时针
    {
        //srd.dir = CCW;
		GPIO_ResetBits(GPIOE,GPIO_Pin_5);
        step = -step;
    }
    else//顺时针
    {
        GPIO_SetBits(GPIOE,GPIO_Pin_5); //PE5输出高 顺时针方向  DRIVER2_DIR
		//srd.dir = CW;
    }
    // 输出电机方向
    //DIR(srd.dir);
    // 配置电机为输出状态
    //status.out_ena = TRUE;
    
    // 如果只移动一步
    if(step == 1)
    {
        // 只移动一步
        srd.accel_count = -1;
        // 减速状态
        srd.run_state = DECEL;
        // 短延时
        srd.step_delay = 20;
        // 配置电机为运行状态
        status.running = TRUE;
        //设置定时器重装值	
        TIM_SetAutoreload(TIM8,srd.step_delay);
        //设置占空比为50%	
        TIM_SetCompare2(TIM8,srd.step_delay>>1);
        //使能定时器	      
        TIM_Cmd(TIM8, ENABLE); 
     }
    // 步数不为零才移动
    else if(step != 0)
    {
   
    // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
    srd.min_delay = 1000000/speed-1;

    // 初始脉冲频率为20Hz
    srd.step_delay = 1000000/20-1;  //f1=20Hz；
		
    // 计算多少脉冲之后达到最大速度的限制
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (((long)(speed-20)/(long)(accel))+1);
		
    // 如果达到最大速度小于0.5个脉冲，我们将四舍五入为0
    // 但实际我们必须移动至少一个脉冲才能达到想要的速度
    if(max_s_lim == 0)
    {
        max_s_lim = 1;
    }

    // 计算多少脉冲之后我们必须开始减速（因为还有减速），由三角形面积即可计算出
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = ((long)step*decel) / (accel+decel)-1;
    // 我们必须加速至少1个脉冲才能才能开始减速.
    if(accel_lim == 0)
    {
        accel_lim = 1;
    }
    // 使用限制条件计算出减速所用的脉冲数（负数表示）
    //srd.decel_val为负数
    if(accel_lim <= max_s_lim)//三角形的情况
    {
        srd.decel_val = accel_lim - step;
    }
    else//梯形的情况
    {
        srd.decel_val = -(((long)(speed-20)/(long)(decel))+1);
    }
    // 当只剩下一步我们必须减速
    if(srd.decel_val == 0)
    {
        srd.decel_val = -1;
    }

    // 计算开始减速时的脉冲数
    srd.decel_start = step + srd.decel_val;
	

    // 如果最大速度很慢，我们就不需要进行加速运动
    if(srd.step_delay <= srd.min_delay)
    {
        srd.step_delay = srd.min_delay;
        srd.run_state = RUN;
    }
    else
   {
        srd.run_state = ACCEL;
   }
   
    // 复位加速度计数值
    srd.accel_count = 0;
    // 复位减速度计数值
    srd.decel_count = 0;
    status.running = TRUE;
    //设置定时器重装值	
    TIM_SetAutoreload(TIM8,srd.step_delay);
    //设置占空比为50%	
    TIM_SetCompare2(TIM8,srd.step_delay>>1);
    //计数器清零
    TIM_SetCounter(TIM8,0);
    //使能定时器	      
    TIM_Cmd(TIM8, ENABLE); 
    }
}
/**
  * @brief  根据运动方向判断步进电机的运行位置
  * @param  inc 运动方向
  * @retval 无
  */
void MSD_StepCounter(signed char inc)
{
  //根据方向判断电机位置
  if(inc == CCW)
  {
    stepPosition--;
  }
  else
  {
    stepPosition++;
  }
}

/******* TIM8更新中断服务程序 *********/
//当TIM8->RCR中的重复计数值减为0时触发中断 也即发送TIM8->RCR+1个脉冲后发生中断
void TIM8_UP_TIM13_IRQHandler(void)
{
	 // 保存下一个延时周期
	unsigned int new_step_delay;
    // 加速过程中最后一次延时.
    static int last_accel_delay;
    // 移动脉冲数计数器
    static unsigned int step_count = 0;
	
	if(TIM_GetITStatus(TIM8,TIM_FLAG_Update)!=RESET)//更新中断
	{
		TIM_ClearITPendingBit(TIM8,TIM_FLAG_Update);//清除更新中断标志位
		TIM_SetAutoreload(TIM8,srd.step_delay);//设定自动重装值	
	    TIM_SetCompare2(TIM8,srd.step_delay>>1);//设置比较值等于重装载值的一半，即占空比为50%
        TIM_SetCounter(TIM8,0);//计数器清零		
		//如果禁止输出，电机则停止运动
       if(status.out_ena != TRUE)
       {
            srd.run_state = STOP;
       }
	   
	switch(srd.run_state) {
		
    case STOP:
		
      step_count = 0;
      //rest = 0;
      TIM8->CCER &= ~(1<<12); //禁止输出
      TIM_Cmd(TIM8, DISABLE);
      status.running = FALSE;
      break;

    case ACCEL:
		
	  TIM_GenerateEvent(TIM8,TIM_EventSource_Update);//产生一个更新事件 重新初始化计数器
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能
      TIM_Cmd(TIM8, ENABLE);  //使能TIM8  
	  //TIM8->CCER |= 1<<12; //使能输出
      MSD_StepCounter(srd.dir);
      step_count++;//记录已经发送的脉冲数
      srd.accel_count++;//记录加速的次数
      new_step_delay = 1000000/(20+a*srd.accel_count)-1;
      //rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
      //检查是够应该开始减速
      if(step_count >= srd.decel_start) {
        srd.accel_count = srd.decel_val;  //srd.decel_val为负数减速用的脉冲数   
        srd.run_state = DECEL;
      }
      //检查是否到达期望的最大速度
      else if(new_step_delay <= srd.min_delay) {
        last_accel_delay = new_step_delay;  //加速过程中最后一次延时=new_step_delay
        new_step_delay = srd.min_delay;  //自动重装载值设置为最大速度
        //rest = 0;
        srd.run_state = RUN;
	
      }
      break;

    case RUN:
		
	  TIM_GenerateEvent(TIM8,TIM_EventSource_Update);//产生一个更新事件 重新初始化计数器
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能
      TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	  //TIM8->CCER |= 1<<12; //使能输出
      MSD_StepCounter(srd.dir);
      step_count++;
      new_step_delay = srd.min_delay;
      //检查是否需要开始减速
      if(step_count >= srd.decel_start) {
        srd.accel_count = srd.decel_val;//把减速的脉冲数给减速计数器（负数）
        new_step_delay = srd.min_delay;
		//以最后一次加速的延时作为开始减速的延时
        //new_step_delay = last_accel_delay;
        srd.run_state = DECEL;
		 
      }
      break;

    case DECEL:
		
	  TIM_GenerateEvent(TIM8,TIM_EventSource_Update);//产生一个更新事件 重新初始化计数器
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能
      TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	  //TIM8->CCER |= 1<<12; //使能输出
      MSD_StepCounter(srd.dir);
      step_count++;
      srd.accel_count++;
	  srd.decel_count++;
	  new_step_delay = 1000000/((1000000/(srd.min_delay+1))-d*srd.decel_count)-1;
	
     
	 if(new_step_delay>=49999)
	 {
	   new_step_delay=49999;
     }
      if(srd.accel_count >= 0){    //若srd.accel_count小于0说明还没减速完
        srd.run_state = STOP;
		  a=0;
		  d=0;
      }
      break;
      }
		
	srd.step_delay = new_step_delay;
	
	}	
}
	


