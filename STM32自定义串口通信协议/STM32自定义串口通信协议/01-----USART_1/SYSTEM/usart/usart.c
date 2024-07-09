#include <stdio.h>
#include "sys.h"
#include "usart.h"	
#include "delay.h"
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif


//�м����

unsigned char N = 0;
unsigned int command,my_chk;	//���յ��������,��У��
unsigned int rec_chk;  //��У�飬���Ͷ�
unsigned int rec_length;  //�����գ����ݳ���
unsigned char commu_err = 4;  //���ݴ���0������Ų��ԣ�1�����ݸ�ʽ���ԣ�2����У�鲻��
unsigned char P2A_Flag = 0;  //��λ��������ARM���ݱ�־λ��0Ϊ��ֵ��1��ʾ��λ������֡��ʽ������ţ���У�鶼��ȷ



//�������ݼĴ���
unsigned char USART_RX_BUF[RX_LENTH];
 	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
  
  //����1��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
  
  //USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
  GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10
  
  //USART1 ��ʼ������
  USART_InitStructure.USART_BaudRate = bound;//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
  USART_ClearFlag(USART1, USART_FLAG_TC);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
  
  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}


void USART1_IRQHandler(void)  //����1�жϷ������
{
  unsigned int rec_data;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж� RXNE��׼���ö�ȡ���յ������ݱ�־λ
  {
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);//��������жϱ�־
    if(N==0) 
    my_chk=0;//��ʼ����У��
    rec_data = USART_ReceiveData(USART1);//��ֵ��ʱ����
    my_chk+=rec_data;//�����У��
    
    //���鿪ʼ��ʶ
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
    
    //���������ʶ
    else if(N==2)
    {
      if(  rec_data!=0x01 && rec_data!=0x02 && rec_data!=0x03 
           && rec_data!=0x04 && rec_data!=0x05 && rec_data!=0x06 ) 
      {
        commu_err = CODE_W;
      }
      else command=rec_data;
    }
    
    //�������ݳ��� 2Byte
    else if(N==3){
      rec_length=rec_data;
    }
    else if(N==4)
    {
      rec_length = (rec_length << 8) + (rec_data); //�ϲ����ݳ���
      if(rec_length>599) 
      {
        commu_err = PROT_W;
      }
    }
    else
    {	
      //��������
      
      if(N<=rec_length+4)
      {
        //�˴��ɸ��ݲ�ͬ����ŵ����ݳ��Ȳ�ͬ�����ò�ͬ��Nֵɸѡ����
        USART_RX_BUF[N-5]=rec_data;  
      
      }
      
      
      //���պ�У�� 2Byte
      else if(N==rec_length+5)  //����У��͸�λ
      {
        rec_chk=rec_data;
      }
      else if(N==rec_length+6)  //����У��͵�λ
      {
        my_chk-=rec_chk;//��ȥ���յ���У��͵�λ
        my_chk-=rec_data;//��ȥ���յ���У��͸�λ����������my_chk�����ɽ��յ���֡ͷһֱ�ۼӵ�У���֮ǰ��֡ͷ+�����+���ݳ���+���ݣ�
        
        rec_chk = (rec_chk << 8) + rec_data;
        
        if(rec_chk != my_chk)  //�жϽ��յ���У�������������У����Ƿ����
        {
          commu_err = SUMM_W;
        }
      }
      //���ս�����ʶ
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
          P2A_Flag = 1;  //��λ���������ݷ���Э�飬������Ͻ�����һ��״̬
        }
      }
    }
    N++;
    if( commu_err == PROT_W || commu_err == CODE_W || commu_err == SUMM_W )
    {
      while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);  //�����ã���ʽ�汾ɾ��
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



void Init_USART_RX_BUF(void)  //������λ�����������ʼ��
{
  unsigned int i;
  for(i=0;i < RX_LENTH;i++)
  {
    USART_RX_BUF[i]=0;
  }
}


