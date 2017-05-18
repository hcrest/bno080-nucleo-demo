// TODO-DW : Header, license

#ifndef USART_H
#define USART_H

#include <stm32f4xx_hal.h>

typedef void (CpltCallback_t)(UART_HandleTypeDef *huart);

void usartRegister(UART_HandleTypeDef *huart, CpltCallback_t *rxCplt, CpltCallback_t *txCplt);
void usartUnregister(UART_HandleTypeDef *huart, CpltCallback_t *rxCplt, CpltCallback_t *txCplt);


#endif
