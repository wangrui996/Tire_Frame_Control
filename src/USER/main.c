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
//毕业设计-智能胎架控制系统程序
//MCU：STM32F407VGT6
//修改日期:2020/05/20
//版本：V2.0

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	

//底盘前进任务
//设置任务优先级
#define FORWARD_TASK_PRIO       			9 
//设置任务堆栈大小
#define FORWARD_STK_SIZE  					128
//任务堆栈	
OS_STK FORWARD_TASK_STK[FORWARD_STK_SIZE];
//任务函数
void forward_task(void *pdata);

//底盘后退任务
//设置任务优先级
#define BACK_TASK_PRIO       			8 
//设置任务堆栈大小
#define BACK_STK_SIZE  					128
//任务堆栈	
OS_STK BACK_TASK_STK[BACK_STK_SIZE];
//任务函数
void back_task(void *pdata);

//丝杠上升任务
//设置任务优先级
#define UP_TASK_PRIO       			7 
//设置任务堆栈大小
#define UP_STK_SIZE  					128
//任务堆栈	
OS_STK UP_TASK_STK[UP_STK_SIZE];
//任务函数
void up_task(void *pdata);

//丝杠下降任务
//设置任务优先级
#define DOWN_TASK_PRIO       			6 
//设置任务堆栈大小
#define DOWN_STK_SIZE  					128
//任务堆栈	
OS_STK DOWN_TASK_STK[DOWN_STK_SIZE];
//任务函数
void down_task(void *pdata);

//丝杠位置控制任务
//设置任务优先级
#define POSITION_TASK_PRIO       			5 
//设置任务堆栈大小
#define POSITION_STK_SIZE  					128
//任务堆栈	
OS_STK POSITION_TASK_STK[POSITION_STK_SIZE];
//任务函数
void position_task(void *pdata);

//压力检测任务
//设置任务优先级
#define PRESSURE_TASK_PRIO       			4 
//设置任务堆栈大小
#define PRESSURE_STK_SIZE  					128
//任务堆栈	
OS_STK PRESSURE_TASK_STK[PRESSURE_STK_SIZE];
//任务函数
void pressure_task(void *pdata);

//位移检测任务
//设置任务优先级
#define HEIGHT_TASK_PRIO       			    3
//设置任务堆栈大小
#define HEIGHT_STK_SIZE  					128
//任务堆栈	
OS_STK HEIGHT_TASK_STK[HEIGHT_STK_SIZE];
//任务函数
void height_task(void *pdata);

//主任务
//设置任务优先级
#define MAIN_TASK_PRIO       			2 
//设置任务堆栈大小
#define MAIN_STK_SIZE  		    	128
//任务堆栈	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//任务函数
void main_task(void *pdata);

//串口通信任务
//设置任务优先级
#define USART_TASK_PRIO       			1 
//设置任务堆栈大小
#define USART_STK_SIZE  		    	128
//任务堆栈	
OS_STK USART_TASK_STK[USART_STK_SIZE];
//任务函数
void usart_task(void *pdata);

//////////////////////////////////////////////////////////////////////////////
OS_EVENT * sem_position;		//信号量指针
OS_EVENT * sem_forward;			//信号量指针
OS_EVENT * sem_back;			//信号量指针
OS_EVENT * msg_command;		    //指令信号量指针
OS_EVENT * Mbox_height;          //高度检测消息邮箱

OS_EVENT * q_msg_adc;			//ADC消息队列

OS_EVENT * q_msg_up;			    //丝杠上升消息队列
OS_EVENT * q_msg_down;			//丝杠下降消息队列
OS_EVENT * q_msg_forward;			//底盘前进消息队列
OS_EVENT * q_msg_back;			//底盘后退消息队列

void * MsgGrp_ADC[4];			//adc消息队列存储地址,最大支持4个消息
void * MsgGrp_up[2];			//adc消息队列存储地址,最大支持4个消息
void * MsgGrp_down[2];			//adc消息队列存储地址,最大支持4个消息
void * MsgGrp_forward[2];		//adc消息队列存储地址,最大支持4个消息
void * MsgGrp_back[2];			//adc消息队列存储地址,最大支持4个消息
//创建消息队列，首先需要定义一个指针数组(用于存放消息邮箱)，然后把各个消息数据缓冲区的首地址存入这个数组中

//系统初始化
void system_init(void)
{
    uart_init(115200);            //初始化串口波特率为115200
	delay_init(168);              //延时初始化
	RS485_Init(38400);
	Encoder_Timer3_Init();
	LED_Init();		        //初始化LED端口---调试用
	TIM1_OPM_RCR_Init(999,168-1); //初始化TIM1为1MHz的计数频率  单脉冲+重复计数模式
    TIM8_OPM_RCR_Init(999,168-1); //初始化TIM8为1MHz的计数频率  单脉冲+重复计数模式
	Driver1_Init();
	Driver_Init();
	Adc_Init(); 				//ADC通道初始化

}

/*****************主函数********************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	system_init();//系统初始化
	
	OSInit();//初始化UCOSII
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
    OSStart();
}
//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata = pdata; 	
	
	q_msg_adc=OSQCreate(&MsgGrp_ADC[0],4);	//创建adc处理任务消息队列
	q_msg_up=OSQCreate(&MsgGrp_up[0],2);	//创建
	q_msg_down=OSQCreate(&MsgGrp_down[0],2);	//创建a
	q_msg_forward=OSQCreate(&MsgGrp_forward[0],2);	//创建
	q_msg_back=OSQCreate(&MsgGrp_back[0],2);	//创建
	
	Mbox_height=OSMboxCreate((void *)0); //建立高度检测消息邮箱
	
	msg_command=OSMboxCreate((void*)0);	//创建消息邮箱
	sem_position=OSSemCreate(0);		//创建信号量
	sem_forward=OSSemCreate(0);		//创建信号量
	sem_back=OSSemCreate(0);		//创建信号量
	
	OSStatInit();		//初始化统计任务.这里会延时1秒钟左右
	
	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断) 
	OSTaskCreate(forward_task,(void *)0,(OS_STK*)&FORWARD_TASK_STK[FORWARD_STK_SIZE-1],FORWARD_TASK_PRIO);
	OSTaskCreate(back_task,(void *)0,(OS_STK*)&BACK_TASK_STK[BACK_STK_SIZE-1],BACK_TASK_PRIO);
	OSTaskCreate(up_task,(void *)0,(OS_STK*)&UP_TASK_STK[UP_STK_SIZE-1],UP_TASK_PRIO);
	OSTaskCreate(down_task,(void *)0,(OS_STK*)&DOWN_TASK_STK[DOWN_STK_SIZE-1],DOWN_TASK_PRIO);
	OSTaskCreate(position_task,(void *)0,(OS_STK*)&POSITION_TASK_STK[POSITION_STK_SIZE-1],POSITION_TASK_PRIO);
	OSTaskCreate(pressure_task,(void *)0,(OS_STK*)&PRESSURE_TASK_STK[PRESSURE_STK_SIZE-1],PRESSURE_TASK_PRIO);
	OSTaskCreate(height_task,(void *)0,(OS_STK*)&HEIGHT_TASK_STK[HEIGHT_STK_SIZE-1],HEIGHT_TASK_PRIO);
	OSTaskCreate(main_task,(void *)0,(OS_STK*)&MAIN_TASK_STK[MAIN_STK_SIZE-1],MAIN_TASK_PRIO);
	OSTaskCreate(usart_task,(void *)0,(OS_STK*)&USART_TASK_STK[USART_STK_SIZE-1],USART_TASK_PRIO);
    
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务
	OS_EXIT_CRITICAL();	//退出临界区(可以被中断打断)
	
}
//底盘前进任务
void forward_task(void *pdata)
{
    int steps;
    int speed;	
	int acceleration = 1;//默认加速度 
    int deceleration = 1;//默认减速度

	int *msg_forward;
    u8 err;
	
	while(1)
	{
		msg_forward=OSQPend(q_msg_forward,0,&err);//等待接收邮箱信息(指针msg指向buffer)
		steps=*msg_forward;
		speed=*(msg_forward+1);
		GPIO_SetBits(GPIOE,GPIO_Pin_5); //PE5输出高 顺时针方向  DRIVER2_DIR
        MSD_Move(steps, acceleration, deceleration, speed);
		
		LED1=!LED1;
		delay_ms(100);
	}
}	
//底盘后退任务
void back_task(void *pdata)
{
	int steps;
    int speed;	
	int acceleration = 1;//默认加速度 
    int deceleration = 1;//默认减速度

	int *msg_back;
	
	u8 err;
	while(1)
	{
		msg_back=OSQPend(q_msg_back,0,&err);//等待接收邮箱信息(指针msg指向buffer)
        steps=-(*msg_back);
		speed=*(msg_back+1);
	    MSD_Move(steps, acceleration, deceleration, speed);
		
		LED1=!LED1;
		delay_ms(10);
	}
}
//丝杠上升任务
void up_task(void *pdata)
{
	int steps;	
	int speed;
	int acceleration = 1;//默认加速度 
    int deceleration = 1;//默认减速度
	
	int *msg_up;
    u8 rs485[3];
	
	u8 err;
	rs485[0]=6;
	rs485[1]=6;
	rs485[2]=6;
	
	
	while(1)
	{
		msg_up=OSQPend(q_msg_up,0,&err);//等待接收邮箱信息(指针msg指向buffer)
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
//丝杠下降任务
void down_task(void *pdata)
{		
	int steps;	
	int speed;
	int acceleration = 1;//默认加速度 
    int deceleration = 1;//默认减速度
  
	int *msg_down;
	
	u8 err;
	while(1)
	{
		msg_down=OSQPend(q_msg_down,0,&err);//等待接收邮箱信息(指针msg指向buffer)
		steps=-(*msg_down);
		speed=*(msg_down+1);
		MSD_Move1(steps, acceleration, deceleration, speed);
		
		LED1=!LED1;
		delay_ms(10);
	}
	
}
	
//丝杠位置控制任务、用于扩展为丝杠运动闭环控制时使用
void position_task(void *pdata)
{
	//u8 err;
	while(1)
	{
		//OSSemPend(sem_position,0,&err);  
		delay_ms(10);
	}
}
//ADC检测任务---获取压力值、锂电池电压值
void pressure_task(void *pdata)
{
	u16 adc1;  //压力adc值
	u16 adc2;  //锂电池电压adc值
	float temp1;//压力--小数
	float temp2;//锂电池电压--小数
	u8 temp1_int;//压力--整数
	u8 temp2_int;//锂电池电压--整数
	u8 temp1_float;//压力--小数部分*100
	u8 temp2_float;//锂电池电压--小数部分*100
	u8 adc[4];
	while(1)
	{
		adc1=Get_Adc1(ADC_Channel_5); 
		adc2=Get_Adc2(ADC_Channel_4);
		//adc[4]=adc1;
		//printf("%d\r\n",adc1);
		//printf("%d\r\n",adc2);
		//adc1=Get_Adc1_Average(ADC_Channel_5,20);//获取通道5的转换值，20次取平均
		//adc2=Get_Adc2_Average(ADC_Channel_4,20);//获取通道4的转换值，20次取平均
		
		temp1=(float)15*3*adc1*(3.3/4095); //获取计算后的带小数的实际压力值--小数
		
		temp2=(float)(104.7/4.7)*adc2*(3.3/4095);   //获取计算后的带小数的实际锂电池电压值--小数

		temp1_int=temp1; //赋值压力整数部分
		temp2_int=temp2; //赋值锂电池电压整数部分
		
		temp1-=temp1_int; //整数部分去掉，留下小数部分，比如3.1111-3=0.1111
		temp2-=temp2_int; //整数部分去掉
	
		temp1*=100; //小数部分乘以100  如上述temp1=11.11		
		temp2*=100; //小数部分乘以100
		
		temp1_float=temp1;//相当于保留两位小数。
		temp2_float=temp2;//相当于保留两位小数。
		
		adc[0]=temp1_int;
		adc[1]=temp1_float;
		adc[2]=temp2_int;
		adc[3]=temp2_float;
		
		
		OSQPost(q_msg_adc,&adc[0]); //发送到队列
		//OSQPost(q_msg_adc,(void*)&temp1_float); //发送到队列
		//OSQPost(q_msg_adc,(void*)&temp2_int); //发送到队列
		//OSQPost(q_msg_adc,(void*)&temp2_float); //发送到队列
		delay_ms(10);
	}
}
//位移检测任务
void height_task(void *pdata)
{
	int height_pulse;
	while(1)
	{
		height_pulse = read_encoder()/4;
		//height_pulse = 256;
		OSMboxPost(Mbox_height,&height_pulse);//通过邮箱发送高度脉冲信息(指针)给485通信任务
		
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
	
	u8 pressure_int;//压力--整数
	u8 li_int;//锂电池电压--整数
	u8 pressure_float;//压力--小数部分*100
	u8 li_float;//锂电池电压--小数部分*100
	u16 height;//高度脉冲
	u8 height_H;//高度脉冲高8位
	u8 height_L;//高度脉冲低8位
	//u16 adc1;
	u8 err1;
	u8 err2;
	u8 rs485_tx_buf[]={237, 06, 87, 32, 0, 0, 0, 0, 0, 0};//这里先假设发送的数据为6位；接收缓冲的数据来源于压力、位移采集模块的处理结果；
	u8 command;
	//rs485[0]=1;
	while(1)
	{
		
		msg_adc=OSQPend(q_msg_adc,0,&err1);//等待接收邮箱信息(指针msg指向buffer)
		pressure_int=*msg_adc;
		pressure_float=*(msg_adc+1);
		li_int=*(msg_adc+2);
		li_float=*(msg_adc+3); 
		//adc1=*(msg_adc+4);		
		
		msg_height=OSMboxPend(Mbox_height,0,&err2);//等待接收邮箱信息(指针msg_height指向位移检测任务中的height_pulse) 
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
	
//485通信任务（Zigbee）
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
	   if(key!=0)//接收到有数据
	   {
		   if(key!=13)
			   RS485_Send_Data(&err1,1);//数据长度错误，返回0x00
		   
		   else 
		   {
		   if(rs485_rx_buf[0]==0xED&&rs485_rx_buf[1]==0x07&&rs485_rx_buf[2]==0x57&&rs485_rx_buf[3]==0x21&&rs485_rx_buf[11]==0x57&&rs485_rx_buf[12]==0x20)//数据格式正确、执行相应命令
		   {
			   RS485_Send_Data(&ture,1);//数据格式正确，返回0x01，并提取数据位
			   switch(rs485_rx_buf[4])  //指令选择位
		     {
			
			    case 1://控制丝杠位置  上升  距离  速度 
                
 				    up[0]=(int)(3553.57*(255*rs485_rx_buf[5] + rs485_rx_buf[6]));
				    up[1]=(int)(3553.57*rs485_rx_buf[7]);
				
				    OSQPost(q_msg_up,&up[0]); //发送到队列
	
				    //RS485_Send_Data(rs485,1);
			        delay_ms(20);
				    
			        break;
			    case 2://控制底盘位置  下降  距离  速度
					down[0]=(int)(3553.57*(255*rs485_rx_buf[5] + rs485_rx_buf[6]));
				    down[1]=(int)(3553.57*rs485_rx_buf[7]);
							
				    OSQPost(q_msg_down,&down[0]); //发送到队列
				    //MSD_Move(steps, acceleration, deceleration, speed);
			        delay_ms(20);
			       
				    break;
			    case 3://控制底盘位置  前进 距离  速度
					forward[0]=abs((int)(1527.88*(255*rs485_rx_buf[8] + rs485_rx_buf[9])));
				    forward[1]=(int)(1527.88*rs485_rx_buf[10]);
						
					OSQPost(q_msg_forward,&forward[0]); //发送到队列
				    
				    break;
				case 4://控制底盘位置  后退  距离  速度
					back[0]=(int)(1527.88*(255*rs485_rx_buf[8] + rs485_rx_buf[9]));
				    back[1]=(int)(1527.88*rs485_rx_buf[10]);
				
					OSQPost(q_msg_back,&back[0]); //发送到队列
				    
				    break;
				case 5://丝杠停止 
					TIM_Cmd(TIM1, DISABLE);
					TIM_SetCounter(TIM1,0);//计数器清零
					
				    break;
				case 6://底盘停止
					TIM_Cmd(TIM8, DISABLE);
				    TIM_SetCounter(TIM8,0);//计数器清零
				
				    break;
		     }	
			     //key = 0;
			   LED0=!LED0;//
		   }
		   else
		     RS485_Send_Data(&err2,1);//报头or报尾错误，返回0x03
		   
	       } 
	   }  
	   
	   key=0;
	   delay_ms(100);   
	}

}

