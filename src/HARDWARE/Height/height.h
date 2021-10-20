#ifndef __HEIGHT_H
#define __HEIGHT_H 
#include "sys.h"

//丝杠位移测量-拉线式编码器数据采集 测试代码	
//修改日期:2020/03/03
//版本：V1.0

/************端口定义**************/
//   PA6  	A相输入
//   PA7    B相输入

/******************************************/

void Encoder_Timer3_Init(void);// 定时器3初始化为编码器
int read_encoder(void);// 读取定时器计数值


#endif
