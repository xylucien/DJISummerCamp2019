/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      主要利用陀螺仪mpu6500，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过mpu6500的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间，提供注释对应的宏定义，关闭DMA，
  *             DR的外部中断的方式.
  * @note       SPI 在陀螺仪初始化的时候需要低于2MHz，之后读取数据需低于20MHz
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

#include "INS_Task.h"

#include "main.h"

#include "IST8310driver.h"
#include "bsp_buzzer.h"
#include "bsp_spi.h"
#include "mpu6500driver.h"
#include "mpu6500driver_middleware.h"
#include "mpu6500reg.h"

#include "cmsis_os.h"

#define IMUWarnBuzzerOn() buzzer_on(95, 10000) //开机陀螺仪校准蜂鸣器

#define IMUWarnBuzzerOFF() buzzer_off() //开机陀螺仪校准蜂鸣器关闭

//宏定义初始化SPI的DMA，同时设置SPI为8位，4分频
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                                     \
  {                                                                            \
    SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM);               \
    SPI5->CR2 |= ((uint16_t)0x0003);                                           \
    SPI5SetSpeedAndDataSize(0x0010, 0x0000);                                   \
  }

#define MPU6500_SPI_DMA_Enable()                                               \
  SPI5_DMA_Enable(DMA_RX_NUM) // 开始一次SPI的DMA传输
//宏定义SPI的DMA传输中断函数以及传输中断标志位
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5

#define IMU_BOARD_INSTALL_SPIN_MATRIX                                          \
  {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, { 0.0f, 0.0f, 1.0f }

//如果使用mpu6500的数据准备外部中断，可以使用任务通知方法唤醒任务
static TaskHandle_t INSTask_Local_Handler;

// DMA的SPI 发送的buf，以INT_STATUS开始连续读取 DMA_RX_NUM大小地址的值
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] = {MPU_INT_STATUS |
                                                          MPU_SPI_READ_MSB};

static uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //保存接收的原始数据

//处理陀螺仪，加速度计，磁力计数据的线性度，零漂
static void IMU_rotation_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3],
                               mpu6500_real_data_t *mpu6500,
                               ist8310_real_data_t *ist8310);

static mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据
static fp32 gyro_scale_factor[3][3] = {
    IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
static fp32 gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};
static fp32 gyro_offset[3] = {0.0f, 0.0f, 0.0f}; //陀螺仪零漂
static fp32 accel_scale_factor[3][3] = {
    IMU_BOARD_INSTALL_SPIN_MATRIX};               //加速度校准线性度
static fp32 accel_offset[3] = {0.0f, 0.0f, 0.0f}; //加速度零漂

static ist8310_real_data_t ist8310_real_data; //转换成国际单位的IST8310数据
static fp32 mag_scale_factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                      {0.0f, 1.0f, 0.0f},
                                      {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
static fp32 mag_offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

void INSTask(void const *pvParameters) {

  //获取当前任务的任务句柄，用于任务通知
  INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));

  osDelay(INS_TASK_INIT_TIME);
  //初始化mpu6500，失败进入死循环
  while (mpu6500_init() != MPU6500_NO_ERROR) {
    ;
  }

  //初始化ist8310，失败进入死循环
  while (ist8310_init() != IST8310_NO_ERROR) {
    ;
  }

  //初始化SPI的DMA传输的方法
  MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);

  while (1) {
    //        //等待外部中断中断唤醒任务
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
    }

    //将读取到的mpu6500原始数据处理成国际单位的数据
    mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET),
                      &mpu6500_real_data);

    //将读取到的ist8310原始数据处理成国际单位的数据
    ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET),
                      &ist8310_real_data);

    //减去零漂以及旋转坐标系
    IMU_rotation_solve(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data,
                       &ist8310_real_data);

    // while(1) end
  }
  // task function end
}

/**
 * @brief          校准陀螺仪
 * @author         RM
 * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[in]      陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
 * @param[in]      陀螺仪的时刻，每次在gyro_offset调用会加1,
 * @retval         返回空
 */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3],
                   uint16_t *time_count) {
  if (*time_count == 0) {
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
  }
  gyro_offset_calc(gyro_offset, INS_gyro, mpu6500_real_data.status, time_count);

  cali_offset[0] = gyro_offset[0];
  cali_offset[1] = gyro_offset[1];
  cali_offset[2] = gyro_offset[2];
  cali_scale[0] = 1.0f;
  cali_scale[1] = 1.0f;
  cali_scale[2] = 1.0f;
}

/**
 * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
 * @author         RM
 * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[in]      陀螺仪的零漂
 * @retval         返回空
 */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]) {
  gyro_cali_offset[0] = cali_offset[0];
  gyro_cali_offset[1] = cali_offset[1];
  gyro_cali_offset[2] = cali_offset[2];
  gyro_offset[0] = gyro_cali_offset[0];
  gyro_offset[1] = gyro_cali_offset[1];
  gyro_offset[2] = gyro_cali_offset[2];
}

extern const fp32 *get_MPU6500_gyro_data_point(void) { return INS_gyro; }
extern const fp32 *get_MPU6500_accel_data_point(void) { return INS_accel; }
extern const fp32 *get_IST8310_mag_data_point(void) { return INS_mag; }

static void IMU_rotation_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3],
                               mpu6500_real_data_t *mpu6500,
                               ist8310_real_data_t *ist8310) {
  for (uint8_t i = 0; i < 3; i++) {
    gyro[i] = mpu6500->gyro[0] * gyro_scale_factor[i][0] +
              mpu6500->gyro[1] * gyro_scale_factor[i][1] +
              mpu6500->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
    accel[i] = mpu6500->accel[0] * accel_scale_factor[i][0] +
               mpu6500->accel[1] * accel_scale_factor[i][1] +
               mpu6500->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
    mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] +
             ist8310->mag[1] * mag_scale_factor[i][1] +
             ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_3) {

  } else if (GPIO_Pin == GPIO_PIN_8) {
    mpu6500_SPI_NS_L();
    MPU6500_SPI_DMA_Enable();
  }
}

void MPU6500_DMA_IRQHandler(void) {
  if (DMA2->HISR & 0x800) {
    mpu6500_SPI_NS_H();
    DMA2->HIFCR = 0xC00;
    //唤醒任务
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INSTask_Local_Handler, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
