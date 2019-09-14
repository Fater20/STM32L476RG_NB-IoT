#include "usart.h"
/* USER CODE BEGIN 0 */

uint8_t Buftemp;

 

uint16_t usart1_recv_len;

uint8_t USART1RECV[MAX_RCV_LEN];

/* USER CODE END 0 */

void MX_USART1_UART_Init(void)
{  
	huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&Buftemp,1);			// Enable the USART1 Interrupt
		
}
	
/* USER CODE BEGIN 1 */
void USART_Clear(UART_HandleTypeDef *huart)
{	
if(huart->Instance == USART1)	
{		usart1_recv_len = 0;		
memset(USART1RECV, 0x0, sizeof(USART1RECV));	
}
else if(huart->Instance == USART2)	
{			
}
}

void USART_IDLECallBack(void)
{	
		unsigned int temp;  
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位  temp = USART1->RDR;  
		//清除状态寄存器RDR	
		temp = temp;  
		HAL_UART_DMAStop(&huart1); //	
}

void GetRcvData(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t rcv_len)
{ 	
	if(huart->Instance == USART1)	
	{			
		if(buf){			
		memcpy(buf,USART1RECV, rcv_len);		
	}		
	USART_Clear(&huart1);	
	}
	else if(huart->Instance == USART2)
	{			
	}
} 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{	
		UNUSED(huart);	
		if(huart->Instance == USART1)	
			{
				//只有第一次中断会调用		
				if(usart1_recv_len==0)
					{		
						USART1RECV[usart1_recv_len++]=Buftemp;
						//第一次中断的数据被写入USART1RECV[0]处			
						__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位			
						__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//使能空闲中断			
						HAL_UART_Receive_DMA(&huart1, USART1RECV+1, MAX_RCV_LEN);	//设置第一次DMA接收缓冲区，		
						}	
			}
	}
