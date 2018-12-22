#include "usart3.h"

u8 USART3_RX_BUF[USART3_REC_LEN];  
u16 USART3_RX_STA=0;       //接收状态标记	

void Initial_UART3(unsigned long baudrate)
{

	 //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = baudrate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);    
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_ClearFlag(USART3,USART_FLAG_TC);	
  USART_Cmd(USART3, ENABLE);  //使能串口3 
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

}

/***********************************/
//函数名称：USART3_IRQHandler
//函数功能：串口中断服务程序
/***********************************/
void USART3_IRQHandler(void)
{
		OSIntEnter();  
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(USART3);
		
		if((USART3_RX_STA&0x8000)==0)
			if(USART3_RX_STA&0x4000)
			{
				if(Res!=0x0a)
				{
					USART3_RX_STA=0;
				}
				else 
				{
					USART3_RX_STA|=0x8000;
				}
			}
			else
			{	
				if(Res==0x0d)USART3_RX_STA|=0x4000;
				else
				{
					USART3_RX_BUF[USART3_RX_STA&0X3FFF]=Res ;
					USART3_RX_STA++;
					if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;
				}		 
			}
		}

}

	