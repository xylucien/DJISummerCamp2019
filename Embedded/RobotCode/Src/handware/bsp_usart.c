#include "bsp_usart.h"
#include "main.h"



void usart2_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    RCC->APB1RSTR |= 0x00020000;
    RCC->APB1RSTR &= ~(0x00020000);

    USART2->BRR = 364;
    USART2->CR1 &= ~(1<<15);
    USART2->CR1|=1<<2;
    USART2->CR1|=1<<3;
    USART2->CR1|=1<<4;

    USART2->CR3|=1<<6;
    USART2->CR3|=1<<7;

    USART2->CR1|=1<<13;


    DMA1_Stream5->CR = 0;

    while(DMA1_Stream5->CR & 0X01)
    {
        ;
    }

    DMA1_Stream5->PAR = (uint32_t) & (USART2->DR);
    DMA1_Stream5->M0AR = (uint32_t)(rx1_buf);
    DMA1_Stream5->M1AR = (uint32_t)(rx2_buf);
    DMA1_Stream5->NDTR = dma_buf_num;

    DMA1_Stream5->CR |= 0 << 6;
    DMA1_Stream5->CR |= 1 << 8;
    DMA1_Stream5->CR |= 0 << 9;
    DMA1_Stream5->CR |= 1 << 10;
    DMA1_Stream5->CR |= 0 << 11;
    DMA1_Stream5->CR |= 0 << 13;
    DMA1_Stream5->CR |= 3 << 16;
    DMA1_Stream5->CR |= 0 << 21;
    DMA1_Stream5->CR |= 0 << 23;
    DMA1_Stream5->CR |= (uint32_t)4 << 25;


    DMA1_Stream5->CR |= 1 << 18;
    DMA1_Stream5->CR |= 1 << 0;



    DMA1_Stream6->CR = 0;

    while(DMA1_Stream6->CR & 0X01)
    {
        ;
    }

    DMA1_Stream6->PAR = (uint32_t) & (USART2->DR);
    DMA1_Stream6->M0AR = (uint32_t)(NULL);
    DMA1_Stream6->NDTR = 0;

    DMA1_Stream6->CR |= 1 << 6;
    DMA1_Stream6->CR |= 0 << 8;
    DMA1_Stream6->CR |= 0 << 9;
    DMA1_Stream6->CR |= 1 << 10;
    DMA1_Stream6->CR |= 0 << 11;
    DMA1_Stream6->CR |= 0 << 13;
    DMA1_Stream6->CR |= 2 << 16;
    DMA1_Stream6->CR |= 0 << 21;
    DMA1_Stream6->CR |= 0 << 23;
    DMA1_Stream6->CR |= (uint32_t)4 << 25;



}



void usart2_dma_tx_enable(uint8_t *tx_buf, uint16_t dma_buf_num)
{
    DMA1_Stream6->CR &= ~(1<<0);
    DMA1->HIFCR |= 1<<21;

    while(DMA1_Stream6->CR & 0X01)
    {
        ;
    }

    DMA1_Stream6->M0AR = (uint32_t)(tx_buf);
    DMA1_Stream6->NDTR = dma_buf_num;

    DMA1_Stream6->CR |= (1<<0);


}










