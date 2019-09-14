/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* Instruction Set ---------------------------------------------------------*/
uint8_t at_stream[]={"AT+NSOCR=STREAM,6,35000,1\r\n"};
uint8_t at_ip[]={"AT+NSOCO=0,118.25.35.43,9999\r\n"};
uint8_t at_send[]={"AT+NSOSD=0,4,00000000\r\n"};
uint8_t at_receive[]={"AT+NSORF=0,1\r\n"};
uint8_t at_restart[]={"AT+NRB\r\n"};
uint8_t at_disconnect[]={"AT+NSOCL=0\r\n"};
uint8_t test[]={"+NSORF:0,118.25.35.43,9999,1,30,0\r\n"};

/*
************************************************************
*	函数名称：	Server_connect
*
*	函数功能：	连接到服务器
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：	无
************************************************************
*/
void Server_connect(void)
{

	HAL_UART_Transmit(&huart1,at_stream,sizeof(at_stream),50);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1,at_ip,sizeof(at_ip),50);
	HAL_Delay(1000);
//	HAL_UART_Transmit(&huart1,test,sizeof(test),50);
//	HAL_Delay(1000);
}

void Server_Send(int *num)
{
	at_send[13]=num[0]/16+48;
	at_send[14]=num[0]%16+48;
	at_send[15]=num[1]/16+48;
	at_send[16]=num[1]%16+48;
	at_send[17]=num[2]/16+48;
	at_send[18]=num[2]%16+48;
	at_send[19]=num[3]/16+48;
	at_send[20]=num[3]%16+48;

	HAL_UART_Transmit(&huart1,at_send,sizeof(at_send),50);
	
	HAL_Delay(3000);
}

void Server_Receive(void)
{
//	HAL_UART_Transmit(&huart2,at_receive,sizeof(at_receive),50);
//	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1,at_receive,sizeof(at_receive),50);
	HAL_Delay(4000);
}

void Restart_m5310(void)
{
	HAL_UART_Transmit(&huart1,at_restart,sizeof(at_restart),50);
	HAL_Delay(10000);
}

void Server_Disconnect(void)
{
	HAL_UART_Transmit(&huart1,at_disconnect,sizeof(at_disconnect),50);
	HAL_Delay(30);
}

