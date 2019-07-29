#include "bsp_remote_control.h"
#include "main.h"
#include "struct_typedef.h"

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
  RCC->APB2RSTR |= 0x10;
  RCC->APB2RSTR &= ~(0x10);

  USART1->BRR = 840;
  USART1->CR1 &= ~(1 << 15);
  USART1->CR1 |= 1 << 2;
  USART1->CR1 |= 1 << 4;
  USART1->CR1 |= 1 << 10;
  USART1->CR1 |= 1 << 13;

  USART1->CR3 |= 1 << 6;

  DMA2_Stream2->CR = 0;

  while (DMA2_Stream2->CR & 0X01) {
    ;
  }

  DMA2_Stream2->PAR = (uint32_t) & (USART1->DR);
  DMA2_Stream2->M0AR = (uint32_t)(rx1_buf);
  DMA2_Stream2->M1AR = (uint32_t)(rx2_buf);
  DMA2_Stream2->NDTR = dma_buf_num;

  DMA2_Stream2->CR |= 0 << 6;
  DMA2_Stream2->CR |= 1 << 8;
  DMA2_Stream2->CR |= 0 << 9;
  DMA2_Stream2->CR |= 1 << 10;
  DMA2_Stream2->CR |= 0 << 11;
  DMA2_Stream2->CR |= 0 << 13;
  DMA2_Stream2->CR |= 3 << 16;
  DMA2_Stream2->CR |= 0 << 21;
  DMA2_Stream2->CR |= 0 << 23;
  DMA2_Stream2->CR |= (uint32_t)4 << 25;

  DMA2_Stream2->CR |= 1 << 18;
  DMA2_Stream2->CR |= 1 << 0;
}
void RC_unable(void) { USART1->CR1 &= ~(1 << 13); }
void RC_restart(uint16_t dma_buf_num) {
  USART1->CR1 &= ~(1 << 13);
  DMA2_Stream2->CR &= ~(1 << 0);

  DMA2_Stream2->NDTR = dma_buf_num;

  DMA2_Stream2->CR |= 1 << 0;
  USART1->CR1 |= 1 << 13;
}
