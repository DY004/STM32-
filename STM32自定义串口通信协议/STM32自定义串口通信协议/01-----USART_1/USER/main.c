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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
    delay_init(168);		//延时初始化
    uart_init(38400);	//串口初始化，波特率为115200
    LED_Init();	//初始化与LED连接的硬件接口
    GPIO_SetBits(GPIOB,GPIO_Pin_10);
    Init_USART_RX_BUF();  //初始化接受寄存器（上位机发送到ARM的数据存储在此数组）
    Init_BUF();  //过程中的变量初始化

    while(1)
    {
        if(P2A_Flag == 1 )
        {

            for(j=0; j<rec_length; j++) //测试用，正式版本删除
            {
                while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
                USART_SendData(USART1,USART_RX_BUF[j]);
            }

            Init_BUF();
            Init_USART_RX_BUF();
        }
    }
}
