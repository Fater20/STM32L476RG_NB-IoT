#ifndef _NB_IOT_H_
#define _NB_IOT_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

void Server_connect(void);
void Server_Send(int *num);
void Server_Receive(void);
void Restart_m5310(void);
void Server_Disconnect(void);

#endif

