#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "motor1.h"
#include "motor2.h"
#include "height.h"
#include "adc.h"
#include "rs485.h"
#include "includes.h"
#include "math.h"
//��ҵ���-����̥�ܿ���ϵͳ����
//MCU��STM32F407VGT6
//�޸�����:2020/05/20
//�汾��V2.0

/////////////////////////UCOSII��������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	

//����ǰ������
//�����������ȼ�
#define FORWARD_TASK_PRIO       			9 
//���������ջ��С
#define FORWARD_STK_SIZE  					128
//�����ջ	
OS_STK FORWARD_TASK_STK[FORWARD_STK_SIZE];
//������
void forward_task(void *pdata);

//���̺�������
//�����������ȼ�
#define BACK_TASK_PRIO       			8 
//���������ջ��С
#define BACK_STK_SIZE  					128
//�����ջ	
OS_STK BACK_TASK_STK[BACK_STK_SIZE];
//������
void back_task(void *pdata);

//˿����������
//�����������ȼ�
#define UP_TASK_PRIO       			7 
//���������ջ��С
#define UP_STK_SIZE  					128
//�����ջ	
OS_STK UP_TASK_STK[UP_STK_SIZE];
//������
void up_task(void *pdata);

//˿���½�����
//�����������ȼ�
#define DOWN_TASK_PRIO       			6 
//���������ջ��С
#define DOWN_STK_SIZE  					128
//�����ջ	
OS_STK DOWN_TASK_STK[DOWN_STK_SIZE];
//������
void down_task(void *pdata);

//˿��λ�ÿ�������
//�����������ȼ�
#define POSITION_TASK_PRIO       			5 
//���������ջ��С
#define POSITION_STK_SIZE  					128
//�����ջ	
OS_STK POSITION_TASK_STK[POSITION_STK_SIZE];
//������
void position_task(void *pdata);

//ѹ���������
//�����������ȼ�
#define PRESSURE_TASK_PRIO       			4 
//���������ջ��С
#define PRESSURE_STK_SIZE  					128
//�����ջ	
OS_STK PRESSURE_TASK_STK[PRESSURE_STK_SIZE];
//������
void pressure_task(void *pdata);

//λ�Ƽ������
//�����������ȼ�
#define HEIGHT_TASK_PRIO       			    3
//���������ջ��С
#define HEIGHT_STK_SIZE  					128
//�����ջ	
OS_STK HEIGHT_TASK_STK[HEIGHT_STK_SIZE];
//������
void height_task(void *pdata);

//������
//�����������ȼ�
#define MAIN_TASK_PRIO       			2 
//���������ջ��С
#define MAIN_STK_SIZE  		    	128
//�����ջ	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//������
void main_task(void *pdata);

//����ͨ������
//�����������ȼ�
#define USART_TASK_PRIO       			1 
//���������ջ��С
#define USART_STK_SIZE  		    	128
//�����ջ	
OS_STK USART_TASK_STK[USART_STK_SIZE];
//������
void usart_task(void *pdata);

//////////////////////////////////////////////////////////////////////////////
OS_EVENT * sem_position;		//�ź���ָ��
OS_EVENT * sem_forward;			//�ź���ָ��
OS_EVENT * sem_back;			//�ź���ָ��
OS_EVENT * msg_command;		    //ָ���ź���ָ��
OS_EVENT * Mbox_height;          //�߶ȼ����Ϣ����

OS_EVENT * q_msg_adc;			//ADC��Ϣ����

OS_EVENT * q_msg_up;			    //˿��������Ϣ����
OS_EVENT * q_msg_down;			//˿���½���Ϣ����
OS_EVENT * q_msg_forward;			//����ǰ����Ϣ����
OS_EVENT * q_msg_back;			//���̺�����Ϣ����

void * MsgGrp_ADC[4];			//adc��Ϣ���д洢��ַ,���֧��4����Ϣ
void * MsgGrp_up[2];			//adc��Ϣ���д洢��ַ,���֧��4����Ϣ
void * MsgGrp_down[2];			//adc��Ϣ���д洢��ַ,���֧��4����Ϣ
void * MsgGrp_forward[2];		//adc��Ϣ���д洢��ַ,���֧��4����Ϣ
void * MsgGrp_back[2];			//adc��Ϣ���д洢��ַ,���֧��4����Ϣ
//������Ϣ���У�������Ҫ����һ��ָ������(���ڴ����Ϣ����)��Ȼ��Ѹ�����Ϣ���ݻ��������׵�ַ�������������

//ϵͳ��ʼ��
void system_init(void)
{
    uart_init(115200);            //��ʼ�����ڲ�����Ϊ115200
	delay_init(168);              //��ʱ��ʼ��
	RS485_Init(38400);
	Encoder_Timer3_Init();
	LED_Init();		        //��ʼ��LED�˿�---������
	TIM1_OPM_RCR_Init(999,168-1); //��ʼ��TIM1Ϊ1MHz�ļ���Ƶ��  ������+�ظ�����ģʽ
    TIM8_OPM_RCR_Init(999,168-1); //��ʼ��TIM8Ϊ1MHz�ļ���Ƶ��  ������+�ظ�����ģʽ
	Driver1_Init();
	Driver_Init();
	Adc_Init(); 				//ADCͨ����ʼ��

}

/*****************������********************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	system_init();//ϵͳ��ʼ��
	
	OSInit();//��ʼ��UCOSII
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
    OSStart();
}
//��ʼ����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata = pdata; 	
	
	q_msg_adc=OSQCreate(&MsgGrp_ADC[0],4);	//����adc����������Ϣ����
	q_msg_up=OSQCreate(&MsgGrp_up[0],2);	//����
	q_msg_down=OSQCreate(&MsgGrp_down[0],2);	//����a
	q_msg_forward=OSQCreate(&MsgGrp_forward[0],2);	//����
	q_msg_back=OSQCreate(&MsgGrp_back[0],2);	//����
	
	Mbox_height=OSMboxCreate((void *)0); //�����߶ȼ����Ϣ����
	
	msg_command=OSMboxCreate((void*)0);	//������Ϣ����
	sem_position=OSSemCreate(0);		//�����ź���
	sem_forward=OSSemCreate(0);		//�����ź���
	sem_back=OSSemCreate(0);		//�����ź���
	
	OSStatInit();		//��ʼ��ͳ������.�������ʱ1��������
	
	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��) 
	OSTaskCreate(forward_task,(void *)0,(OS_STK*)&FORWARD_TASK_STK[FORWARD_STK_SIZE-1],FORWARD_TASK_PRIO);
	OSTaskCreate(back_task,(void *)0,(OS_STK*)&BACK_TASK_STK[BACK_STK_SIZE-1],BACK_TASK_PRIO);
	OSTaskCreate(up_task,(void *)0,(OS_STK*)&UP_TASK_STK[UP_STK_SIZE-1],UP_TASK_PRIO);
	OSTaskCreate(down_task,(void *)0,(OS_STK*)&DOWN_TASK_STK[DOWN_STK_SIZE-1],DOWN_TASK_PRIO);
	OSTaskCreate(position_task,(void *)0,(OS_STK*)&POSITION_TASK_STK[POSITION_STK_SIZE-1],POSITION_TASK_PRIO);
	OSTaskCreate(pressure_task,(void *)0,(OS_STK*)&PRESSURE_TASK_STK[PRESSURE_STK_SIZE-1],PRESSURE_TASK_PRIO);
	OSTaskCreate(height_task,(void *)0,(OS_STK*)&HEIGHT_TASK_STK[HEIGHT_STK_SIZE-1],HEIGHT_TASK_PRIO);
	OSTaskCreate(main_task,(void *)0,(OS_STK*)&MAIN_TASK_STK[MAIN_STK_SIZE-1],MAIN_TASK_PRIO);
	OSTaskCreate(usart_task,(void *)0,(OS_STK*)&USART_TASK_STK[USART_STK_SIZE-1],USART_TASK_PRIO);
    
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����
	OS_EXIT_CRITICAL();	//�˳��ٽ���(���Ա��жϴ��)
	
}
//����ǰ������
void forward_task(void *pdata)
{
    int steps;
    int speed;	
	int acceleration = 1;//Ĭ�ϼ��ٶ� 
    int deceleration = 1;//Ĭ�ϼ��ٶ�

	int *msg_forward;
    u8 err;
	
	while(1)
	{
		msg_forward=OSQPend(q_msg_forward,0,&err);//�ȴ�����������Ϣ(ָ��msgָ��buffer)
		steps=*msg_forward;
		speed=*(msg_forward+1);
		GPIO_SetBits(GPIOE,GPIO_Pin_5); //PE5����� ˳ʱ�뷽��  DRIVER2_DIR
        MSD_Move(steps, acceleration, deceleration, speed);
		
		LED1=!LED1;
		delay_ms(100);
	}
}	
//���̺�������
void back_task(void *pdata)
{
	int steps;
    int speed;	
	int acceleration = 1;//Ĭ�ϼ��ٶ� 
    int deceleration = 1;//Ĭ�ϼ��ٶ�

	int *msg_back;
	
	u8 err;
	while(1)
	{
		msg_back=OSQPend(q_msg_back,0,&err);//�ȴ�����������Ϣ(ָ��msgָ��buffer)
        steps=-(*msg_back);
		speed=*(msg_back+1);
	    MSD_Move(steps, acceleration, deceleration, speed);
		
		LED1=!LED1;
		delay_ms(10);
	}
}
//˿����������
void up_task(void *pdata)
{
	int steps;	
	int speed;
	int acceleration = 1;//Ĭ�ϼ��ٶ� 
    int deceleration = 1;//Ĭ�ϼ��ٶ�
	
	int *msg_up;
    u8 rs485[3];
	
	u8 err;
	rs485[0]=6;
	rs485[1]=6;
	rs485[2]=6;
	
	
	while(1)
	{
		msg_up=OSQPend(q_msg_up,0,&err);//�ȴ�����������Ϣ(ָ��msgָ��buffer)
		steps=*msg_up;
		speed=*(msg_up+1);
		
		MSD_Move1(steps, acceleration, deceleration, speed);
		//rs485[1]=speed/255;
		//rs485[2]=speed%255;
		//RS485_Send_Data(rs485,3);
		LED1=!LED1;
		delay_ms(10);
	}
}
//˿���½�����
void down_task(void *pdata)
{		
	int steps;	
	int speed;
	int acceleration = 1;//Ĭ�ϼ��ٶ� 
    int deceleration = 1;//Ĭ�ϼ��ٶ�
  
	int *msg_down;
	
	u8 err;
	while(1)
	{
		msg_down=OSQPend(q_msg_down,0,&err);//�ȴ�����������Ϣ(ָ��msgָ��buffer)
		steps=-(*msg_down);
		speed=*(msg_down+1);
		MSD_Move1(steps, acceleration, deceleration, speed);
		
		LED1=!LED1;
		delay_ms(10);
	}
	
}
	
//˿��λ�ÿ�������������չΪ˿���˶��ջ�����ʱʹ��
void position_task(void *pdata)
{
	//u8 err;
	while(1)
	{
		//OSSemPend(sem_position,0,&err);  
		delay_ms(10);
	}
}
//ADC�������---��ȡѹ��ֵ��﮵�ص�ѹֵ
void pressure_task(void *pdata)
{
	u16 adc1;  //ѹ��adcֵ
	u16 adc2;  //﮵�ص�ѹadcֵ
	float temp1;//ѹ��--С��
	float temp2;//﮵�ص�ѹ--С��
	u8 temp1_int;//ѹ��--����
	u8 temp2_int;//﮵�ص�ѹ--����
	u8 temp1_float;//ѹ��--С������*100
	u8 temp2_float;//﮵�ص�ѹ--С������*100
	u8 adc[4];
	while(1)
	{
		adc1=Get_Adc1(ADC_Channel_5); 
		adc2=Get_Adc2(ADC_Channel_4);
		//adc[4]=adc1;
		//printf("%d\r\n",adc1);
		//printf("%d\r\n",adc2);
		//adc1=Get_Adc1_Average(ADC_Channel_5,20);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
		//adc2=Get_Adc2_Average(ADC_Channel_4,20);//��ȡͨ��4��ת��ֵ��20��ȡƽ��
		
		temp1=(float)15*3*adc1*(3.3/4095); //��ȡ�����Ĵ�С����ʵ��ѹ��ֵ--С��
		
		temp2=(float)(104.7/4.7)*adc2*(3.3/4095);   //��ȡ�����Ĵ�С����ʵ��﮵�ص�ѹֵ--С��

		temp1_int=temp1; //��ֵѹ����������
		temp2_int=temp2; //��ֵ﮵�ص�ѹ��������
		
		temp1-=temp1_int; //��������ȥ��������С�����֣�����3.1111-3=0.1111
		temp2-=temp2_int; //��������ȥ��
	
		temp1*=100; //С�����ֳ���100  ������temp1=11.11		
		temp2*=100; //С�����ֳ���100
		
		temp1_float=temp1;//�൱�ڱ�����λС����
		temp2_float=temp2;//�൱�ڱ�����λС����
		
		adc[0]=temp1_int;
		adc[1]=temp1_float;
		adc[2]=temp2_int;
		adc[3]=temp2_float;
		
		
		OSQPost(q_msg_adc,&adc[0]); //���͵�����
		//OSQPost(q_msg_adc,(void*)&temp1_float); //���͵�����
		//OSQPost(q_msg_adc,(void*)&temp2_int); //���͵�����
		//OSQPost(q_msg_adc,(void*)&temp2_float); //���͵�����
		delay_ms(10);
	}
}
//λ�Ƽ������
void height_task(void *pdata)
{
	int height_pulse;
	while(1)
	{
		height_pulse = read_encoder()/4;
		//height_pulse = 256;
		OSMboxPost(Mbox_height,&height_pulse);//ͨ�����䷢�͸߶�������Ϣ(ָ��)��485ͨ������
		
		//LED0=!LED0;
		//LED1=!LED1;
		delay_ms(10);
	}
}

void main_task(void *pdata)
{
	//u8 i=0;
    //u8 key;
	u8 *msg_adc;
	u16 *msg_height;
	
	u8 pressure_int;//ѹ��--����
	u8 li_int;//﮵�ص�ѹ--����
	u8 pressure_float;//ѹ��--С������*100
	u8 li_float;//﮵�ص�ѹ--С������*100
	u16 height;//�߶�����
	u8 height_H;//�߶������8λ
	u8 height_L;//�߶������8λ
	//u16 adc1;
	u8 err1;
	u8 err2;
	u8 rs485_tx_buf[]={237, 06, 87, 32, 0, 0, 0, 0, 0, 0};//�����ȼ��跢�͵�����Ϊ6λ�����ջ����������Դ��ѹ����λ�Ʋɼ�ģ��Ĵ�������
	u8 command;
	//rs485[0]=1;
	while(1)
	{
		
		msg_adc=OSQPend(q_msg_adc,0,&err1);//�ȴ�����������Ϣ(ָ��msgָ��buffer)
		pressure_int=*msg_adc;
		pressure_float=*(msg_adc+1);
		li_int=*(msg_adc+2);
		li_float=*(msg_adc+3); 
		//adc1=*(msg_adc+4);		
		
		msg_height=OSMboxPend(Mbox_height,0,&err2);//�ȴ�����������Ϣ(ָ��msg_heightָ��λ�Ƽ�������е�height_pulse) 
		height=*msg_height;
		height_H = height / 255;
		height_L = height % 255;
		
		rs485_tx_buf[4] = pressure_int;
		rs485_tx_buf[5] = pressure_float;
		rs485_tx_buf[6] = li_int;
		rs485_tx_buf[7] = li_float;
		rs485_tx_buf[8] = height_H;
		rs485_tx_buf[9] = height_L;
		RS485_Send_Data(rs485_tx_buf,10);
		delay_ms(2000);
		
	}	
        
}
	
//485ͨ������Zigbee��
void usart_task(void *pdata)
{
	u8 key;
	u8 err1=0x00;
	u8 ture=0x01;
	u8 err2=0x03;
	u8 rs485_rx_buf[13];
	u8 rs[1];
	int up[2];
	int down[2];
	int forward[2];
	int back[2];
	
	rs[0]=6;
	while(1)
	{
	   key=0;
	   //RS485_Send_Data(&key,1);
		delay_ms(1000);
       //GPIO_ResetBits(GPIOB,GPIO_Pin_8);		
	   RS485_Receive_Data(rs485_rx_buf,&key);
	   if(key!=0)//���յ�������
	   {
		   if(key!=13)
			   RS485_Send_Data(&err1,1);//���ݳ��ȴ��󣬷���0x00
		   
		   else 
		   {
		   if(rs485_rx_buf[0]==0xED&&rs485_rx_buf[1]==0x07&&rs485_rx_buf[2]==0x57&&rs485_rx_buf[3]==0x21&&rs485_rx_buf[11]==0x57&&rs485_rx_buf[12]==0x20)//���ݸ�ʽ��ȷ��ִ����Ӧ����
		   {
			   RS485_Send_Data(&ture,1);//���ݸ�ʽ��ȷ������0x01������ȡ����λ
			   switch(rs485_rx_buf[4])  //ָ��ѡ��λ
		     {
			
			    case 1://����˿��λ��  ����  ����  �ٶ� 
                
 				    up[0]=(int)(3553.57*(255*rs485_rx_buf[5] + rs485_rx_buf[6]));
				    up[1]=(int)(3553.57*rs485_rx_buf[7]);
				
				    OSQPost(q_msg_up,&up[0]); //���͵�����
	
				    //RS485_Send_Data(rs485,1);
			        delay_ms(20);
				    
			        break;
			    case 2://���Ƶ���λ��  �½�  ����  �ٶ�
					down[0]=(int)(3553.57*(255*rs485_rx_buf[5] + rs485_rx_buf[6]));
				    down[1]=(int)(3553.57*rs485_rx_buf[7]);
							
				    OSQPost(q_msg_down,&down[0]); //���͵�����
				    //MSD_Move(steps, acceleration, deceleration, speed);
			        delay_ms(20);
			       
				    break;
			    case 3://���Ƶ���λ��  ǰ�� ����  �ٶ�
					forward[0]=abs((int)(1527.88*(255*rs485_rx_buf[8] + rs485_rx_buf[9])));
				    forward[1]=(int)(1527.88*rs485_rx_buf[10]);
						
					OSQPost(q_msg_forward,&forward[0]); //���͵�����
				    
				    break;
				case 4://���Ƶ���λ��  ����  ����  �ٶ�
					back[0]=(int)(1527.88*(255*rs485_rx_buf[8] + rs485_rx_buf[9]));
				    back[1]=(int)(1527.88*rs485_rx_buf[10]);
				
					OSQPost(q_msg_back,&back[0]); //���͵�����
				    
				    break;
				case 5://˿��ֹͣ 
					TIM_Cmd(TIM1, DISABLE);
					TIM_SetCounter(TIM1,0);//����������
					
				    break;
				case 6://����ֹͣ
					TIM_Cmd(TIM8, DISABLE);
				    TIM_SetCounter(TIM8,0);//����������
				
				    break;
		     }	
			     //key = 0;
			   LED0=!LED0;//
		   }
		   else
		     RS485_Send_Data(&err2,1);//��ͷor��β���󣬷���0x03
		   
	       } 
	   }  
	   
	   key=0;
	   delay_ms(100);   
	}

}

