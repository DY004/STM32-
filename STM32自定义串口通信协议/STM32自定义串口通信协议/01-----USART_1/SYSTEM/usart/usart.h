#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	

//���ر�ʶ
#define CODE_W		0x01//����Ų���
#define PROT_W		0x02//֡��ʽ����
#define SUMM_W 		0x03//��У�鲻��

//�洢�������鳤��
#define RX_LENTH	600  	
 
    	
void Init_USART_RX_BUF(void);
void Init_BUF(void);
void uart_init(u32 bound);



#endif


