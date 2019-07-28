#include "bsp_spi.h"
#include "struct_typedef.h"
#include "main.h"



void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize)
{
    SPI5->CR1 &= 0xF7C7;
    SPI5->CR1 |= Speed;
    SPI5->CR1 |= DataSize;
    SPI5->CR1 |= 1 << 6;
}


void SPI5_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    DMA2_Stream5->CR = 0;

    while(DMA2_Stream5->CR & 0X01)
    {
        ;
    }
    DMA2->HIFCR = 0xC00;


    DMA2_Stream5->PAR = (uint32_t) & (SPI5->DR);
    DMA2_Stream5->M0AR = rx_buf;
    DMA2_Stream5->NDTR = num;

    DMA2_Stream5->CR |= 0 << 6;
    DMA2_Stream5->CR |= 0 << 8;
    DMA2_Stream5->CR |= 0 << 9;
    DMA2_Stream5->CR |= 1 << 10;
    DMA2_Stream5->CR |= 0 << 11;
    DMA2_Stream5->CR |= 0 << 13;
    DMA2_Stream5->CR |= 3 << 16;
    DMA2_Stream5->CR |= 0 << 21;
    DMA2_Stream5->CR |= 0 << 23;
    DMA2_Stream5->CR |= (uint32_t)7 << 25;//通道选择

    DMA2_Stream5->CR |= 0x10;


    DMA2_Stream4->CR = 0;
    while(DMA2_Stream4->CR & 0X01)
    {
        ;
    }
    DMA2->HIFCR = 0x30;

    DMA2_Stream4->PAR = (uint32_t) & (SPI5->DR);
    DMA2_Stream4->M0AR = tx_buf;
    DMA2_Stream4->NDTR = num;

    DMA2_Stream4->CR |= 1 << 6;
    DMA2_Stream4->CR |= 0 << 8;
    DMA2_Stream4->CR |= 0 << 9;
    DMA2_Stream4->CR |= 1 << 10;
    DMA2_Stream4->CR |= 0 << 11;
    DMA2_Stream4->CR |= 0 << 13;
    DMA2_Stream4->CR |= 2 << 16;
    DMA2_Stream4->CR |= 0 << 21;
    DMA2_Stream4->CR |= 0 << 23;
    DMA2_Stream4->CR |= (uint32_t)2 << 25;//通道选择


}

void SPI5_DMA_Enable(uint16_t ndtr)
{
    DMA2_Stream4->CR &= ~(1 << 0);
    DMA2_Stream5->CR &= ~(1 << 0);

    while(DMA2_Stream4->CR & 0X01)
    {
        ;
    }
    while(DMA2_Stream5->CR & 0X01)
    {
        ;
    }

    DMA2->HIFCR = 0xC00;
    DMA2->HIFCR = 0x30;

    DMA2_Stream4->NDTR = ndtr;
    DMA2_Stream5->NDTR = ndtr;

    DMA2_Stream4->CR |= (1 << 0);
    DMA2_Stream5->CR |= (1 << 0);
}


