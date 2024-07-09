#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"


extern unsigned char P2A_Flag;
extern unsigned int rec_length;
extern unsigned char USART_RX_BUF[RX_LENTH];
unsigned int j;
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
    delay_init(168);		//��ʱ��ʼ��
    uart_init(38400);	//���ڳ�ʼ����������Ϊ115200
    LED_Init();	//��ʼ����LED���ӵ�Ӳ���ӿ�
    GPIO_SetBits(GPIOB,GPIO_Pin_10);
    Init_USART_RX_BUF();  //��ʼ�����ܼĴ�������λ�����͵�ARM�����ݴ洢�ڴ����飩
    Init_BUF();  //�����еı�����ʼ��

    while(1)
    {
        if(P2A_Flag == 1 )
        {

            for(j=0; j<rec_length; j++) //�����ã���ʽ�汾ɾ��
            {
                while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
                USART_SendData(USART1,USART_RX_BUF[j]);
            }

            Init_BUF();
            Init_USART_RX_BUF();
        }
    }
}
