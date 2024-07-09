#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	

//返回标识
#define CODE_W		0x01//命令号不对
#define PROT_W		0x02//帧格式不对
#define SUMM_W 		0x03//和校验不对

//存储数据数组长度
#define RX_LENTH	600  	
 
    	
void Init_USART_RX_BUF(void);
void Init_BUF(void);
void uart_init(u32 bound);



#endif


