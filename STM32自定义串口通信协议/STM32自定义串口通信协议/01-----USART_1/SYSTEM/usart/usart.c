#include <stdio.h>
#include "sys.h"
#include "usart.h"	
#include "delay.h"
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif


//中间变量

unsigned char N = 0;
unsigned int command,my_chk;	//接收到的命令号,和校验
unsigned int rec_chk;  //和校验，发送端
unsigned int rec_length;  //（接收）数据长度
unsigned char commu_err = 4;  //数据错误，0→命令号不对；1→数据格式不对；2→和校验不对
unsigned char P2A_Flag = 0;  //上位机发送至ARM数据标志位，0为初值，1表示上位机发送帧格式，命令号，和校验都正确



//接收数据寄存器
unsigned char USART_RX_BUF[RX_LENTH];
 	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
  
  //串口1对应引脚复用映射
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = bound;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  
  USART_Cmd(USART1, ENABLE);  //使能串口1 
  USART_ClearFlag(USART1, USART_FLAG_TC);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


void USART1_IRQHandler(void)  //串口1中断服务程序
{
  unsigned int rec_data;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断 RXNE是准备好读取接收到的数据标志位
  {
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除接受中断标志
    if(N==0) 
    my_chk=0;//初始化和校验
    rec_data = USART_ReceiveData(USART1);//赋值临时变量
    my_chk+=rec_data;//计算和校验
    
    //检验开始标识
    if(N==0)
    {
      if(rec_data!=0xC1) 
      {
        commu_err = PROT_W;
      }
    }
    else if(N==1)
    {
      if(rec_data!=0XD2) 
      {
        commu_err = PROT_W;
      }
    }
    
    //检验命令标识
    else if(N==2)
    {
      if(  rec_data!=0x01 && rec_data!=0x02 && rec_data!=0x03 
           && rec_data!=0x04 && rec_data!=0x05 && rec_data!=0x06 ) 
      {
        commu_err = CODE_W;
      }
      else command=rec_data;
    }
    
    //接收数据长度 2Byte
    else if(N==3){
      rec_length=rec_data;
    }
    else if(N==4)
    {
      rec_length = (rec_length << 8) + (rec_data); //合并数据长度
      if(rec_length>599) 
      {
        commu_err = PROT_W;
      }
    }
    else
    {	
      //接收数据
      
      if(N<=rec_length+4)
      {
        //此处可根据不同命令号的数据长度不同，设置不同的N值筛选条件
        USART_RX_BUF[N-5]=rec_data;  
      
      }
      
      
      //接收和校验 2Byte
      else if(N==rec_length+5)  //接收校验和高位
      {
        rec_chk=rec_data;
      }
      else if(N==rec_length+6)  //接收校验和低位
      {
        my_chk-=rec_chk;//减去接收到的校验和低位
        my_chk-=rec_data;//减去接收到的校验和高位，运行完后的my_chk就是由接收到的帧头一直累加到校验和之前（帧头+命令号+数据长度+数据）
        
        rec_chk = (rec_chk << 8) + rec_data;
        
        if(rec_chk != my_chk)  //判断接收到的校验和与计算出来的校验和是否相等
        {
          commu_err = SUMM_W;
        }
      }
      //接收结束标识
      else if(N==rec_length+7)
      {
        if(rec_data!=0xE3) 
        {
          commu_err = PROT_W;
        }
      }
      else if(N==rec_length+8)
      {
        if(rec_data!=0xF4) 
        {
          commu_err = PROT_W;
        }
        else 
        {
          P2A_Flag = 1;  //上位机发送数据符合协议，接收完毕进入下一个状态
        }
      }
    }
    N++;
    if( commu_err == PROT_W || commu_err == CODE_W || commu_err == SUMM_W )
    {
      while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);  //测试用，正式版本删除
      USART_SendData(USART1,commu_err);  
      Init_USART_RX_BUF();
      Init_BUF();	
    }
    
  }	
}


void Init_BUF(void)
{
  my_chk = 0;
  command = 0;
  rec_chk = 0;
  rec_length=0;
  commu_err=4;
  P2A_Flag = 0;
  N = 0;
}



void Init_USART_RX_BUF(void)  //接收上位机数据数组初始化
{
  unsigned int i;
  for(i=0;i < RX_LENTH;i++)
  {
    USART_RX_BUF[i]=0;
  }
}


