#include "height.h"

//˿��λ�Ʋ���-����ʽ���������ݲɼ� ���Դ���	
//�޸�����:2020/03/03
//�汾��V1.0

//˵����������ε�ƫ������û���λ����У��

//Timer3_ch1:PA6 
//Timer3_ch2:PA7
void Encoder_Timer3_Init(void)//��ʱ��3��ʼ��Ϊ������
{
	//GPIO�ڳ�ʼ��    PA6��PA7����Ϊ��ʱ��3
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//������A��B����������PA6��PA7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        //����ģʽ
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; ��������ģʽ����
	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;      //��©���
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;    
	GPIO_Init(GPIOA, &GPIO_InitStructure);            //��ʼ��GPIOA
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);//GPIOA6����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);//GPIOA7����Ϊ��ʱ��3
	
	//��ʱ����ʼ��    Timer3��ʼ��Ϊ������ģʽ
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//TIM3ʱ��ʹ�� 
	
	TIM_DeInit(TIM3);//TIM3��λ
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //Ԥ��Ƶϵ��������������ΪTIM3ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ   δ��ƵǰΪ84M��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = 65535;//0xffff �Զ���װ��ֵ���ۼ�65536������������»����жϣ�  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ� TIM_CKD_DIV1=0 ʱ�ӷ�Ƶ ����Ƶ
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //��ʼ��TIM3
	
	//����TIM3Ϊ������ģʽ  TI1��TI2����������TIM_ICPolarity_Rising�����з���
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); //����Ϊ������ģʽ
	
	//���벶��
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10; //�˲� �����Լ����ã���ʹ��Ĭ��ֵ
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	//TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM���жϱ�־λ/������±�־λ
    //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//���ж� ����ж� 
	
	TIM_SetCounter(TIM3, 0);//����������
	TIM_Cmd(TIM3, ENABLE);

}

// ��ȡ��ʱ������ֵ
int read_encoder(void)
{
	
	int encoder_num;
	encoder_num = (int)((int16_t)(TIM3->CNT)); // ����������Ҫע����������
	//TIM_SetCounter(TIM3, 0);
	return encoder_num;
	
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
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

