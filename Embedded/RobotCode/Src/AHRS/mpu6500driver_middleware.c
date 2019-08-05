/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       mpu6500middleware.c/h
  * @brief      mpu6500磁力计中间层，完成mpu6500的通信函数,延时函数。
  *             
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "mpu6500driver_middleware.h"
#include "main.h"
#include "bsp_delay.h"
#include "cmsis_os.h"

#if defined(MPU6500_USE_SPI)
extern SPI_HandleTypeDef hspi5;

#elif defined(MPU6500_USE_IIC)

#endif

void mpu6500_GPIO_init(void)
{

#if defined(MPU6500_USE_SPI)

#elif defined(MPU6500_USE_IIC)

#endif
}

void mpu6500_com_init(void)
{

#if defined(MPU6500_USE_SPI)


#elif defined(MPU6500_USE_IIC)

#else

#error "Please select the communication of MPU6500"

#endif
}

void mpu6500_middleware_delay_ms(uint16_t ms)
{
    osDelay(ms);
}
void mpu6500_middleware_delay_us(uint32_t us)
{
    delay_us(us);
}

#if defined(MPU6500_USE_SPI)

void mpu6500_SPI_NS_H(void)
{
    HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin, GPIO_PIN_SET);
}
void mpu6500_SPI_NS_L(void)
{
    HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin, GPIO_PIN_RESET);
}

static uint8_t mpu6500_SPI_read_write_byte(uint8_t TxData)
{
    uint8_t res;
    HAL_SPI_TransmitReceive(&hspi5, &TxData, &res, 1, 10);
    return res;
}
void mpu6500_write_single_reg(uint8_t reg, uint8_t data)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg);
    mpu6500_SPI_read_write_byte(data);
    mpu6500_SPI_NS_H();
}

uint8_t mpu6500_read_single_reg(uint8_t reg)
{
    uint8_t res;
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
    res = mpu6500_SPI_read_write_byte(0xFF);
    mpu6500_SPI_NS_H();
    return res;
}

void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            mpu6500_SPI_read_write_byte(*buf);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            *buf = mpu6500_SPI_read_write_byte(0xFF);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

#elif defined(MPU6500_USE_IIC)

#endif
