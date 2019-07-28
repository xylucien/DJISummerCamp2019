#ifndef BSP_USART_H
#define BSP_USART_H
#include "main.h"






extern void usart2_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart2_dma_tx_enable(uint8_t *tx_buf, uint16_t dma_buf_num);
#endif
