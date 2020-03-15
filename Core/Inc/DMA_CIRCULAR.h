
#ifndef _DMA_CIRCULAR_H
#define _DMA_CIRCULAR_H

#include "stm32l4xx_hal.h"

#define DMA_RX_BUFFER_SIZE          256
#define UART_BUFFER_SIZE            512

extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];

void USART_IrqHandler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);
void DMA_IrqHandler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

#endif // _DMA_CIRCULAR_H

