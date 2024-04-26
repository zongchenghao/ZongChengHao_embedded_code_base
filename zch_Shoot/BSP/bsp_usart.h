#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"

int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);

#endif
