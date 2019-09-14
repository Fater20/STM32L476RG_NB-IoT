/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus 
	extern "C" {
#endif 
	
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h" 
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */ 
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3; 

/* USER CODE BEGIN Private defines */
#define MAX_RCV_LEN 1024
extern uint8_t USART1RECV[MAX_RCV_LEN];	 //´®¿Ú1
/* USER CODE END Private defines */ 

extern void _Error_Handler(char *, int); 
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void); 

/* USER CODE BEGIN Prototypes */
/* Çå¿Õ*/
void USART_Clear(UART_HandleTypeDef *huart); 
extern void USART_IDLECallBack(void); 
uint16_t GetRcvNum(UART_HandleTypeDef *huart);
extern void GetRcvData(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t rcv_len);
void USART_Write(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t len);
/* USER CODE END Prototypes */ 
#ifdef __cplusplus
}
#endif
#endif 
/*__ usart_H */
