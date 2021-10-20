#include "height.h"

//丝杠位移测量-拉线式编码器数据采集 测试代码	
//修改日期:2020/03/03
//版本：V1.0

//说明：反复多次的偏差可以用回零位进行校正

//Timer3_ch1:PA6 
//Timer3_ch2:PA7
void Encoder_Timer3_Init(void)//定时器3初始化为编码器
{
	//GPIO口初始化    PA6、PA7复用为定时器3
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//编码器A、B相输入引脚PA6、PA7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        //复用模式
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; 浮空输入模式？？
	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;      //开漏输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;    
	GPIO_Init(GPIOA, &GPIO_InitStructure);            //初始化GPIOA
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);//GPIOA6复用为定时器3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);//GPIOA7复用为定时器3
	
	//定时器初始化    Timer3初始化为编码器模式
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//TIM3时钟使能 
	
	TIM_DeInit(TIM3);//TIM3复位
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //预分频系数：设置用来作为TIM3时钟频率除数的预分频值   未分频前为84M？
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 65535;//0xffff 自动重装载值、累计65536个后产生个更新或者中断；  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割 TIM_CKD_DIV1=0 时钟分频 不分频
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化TIM3
	
	//设置TIM3为编码器模式  TI1与TI2处均计数，TIM_ICPolarity_Rising不进行反相
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); //配置为编码器模式
	
	//输入捕获
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10; //滤波 可以自己设置，或使用默认值
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	//TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM的中断标志位/清除更新标志位
    //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//打开中断 溢出中断 
	
	TIM_SetCounter(TIM3, 0);//计数器清零
	TIM_Cmd(TIM3, ENABLE);

}

// 读取定时器计数值
int read_encoder(void)
{
	
	int encoder_num;
	encoder_num = (int)((int16_t)(TIM3->CNT)); // 这里尤其需要注意数据类型
	//TIM_SetCounter(TIM3, 0);
	return encoder_num;
	
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
         {
           case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
                 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;        
                 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;        
                 default:  Encoder_TIM=0;
         }
                return Encoder_TIM;
}

