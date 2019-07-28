#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "struct_typedef.h"




extern void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize);
extern void SPI5_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void SPI5_DMA_Enable(uint16_t ndtr);
#endif



