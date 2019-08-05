/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      ��Ҫ����������mpu6500��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��mpu6500��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ�䣬�ṩע�Ͷ�Ӧ�ĺ궨�壬�ر�DMA��
  *             DR���ⲿ�жϵķ�ʽ.
  * @note       SPI �������ǳ�ʼ����ʱ����Ҫ����2MHz��֮���ȡ���������20MHz
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "INS_Task.h"
#include "AHRS.h"
#include "main.h"

#include "IST8310driver.h"
#include "bsp_buzzer.h"
#include "bsp_spi.h"
#include "mpu6500driver.h"
#include "mpu6500driver_middleware.h"
#include "mpu6500reg.h"

#include "cmsis_os.h"


#define IMUWarnBuzzerOn() \
  buzzer_on(95, 10000)  //����������У׼������

#define IMUWarnBuzzerOFF() \
  buzzer_off()  //����������У׼�������ر�

//�궨���ʼ��SPI��DMA��ͬʱ����SPIΪ8λ��4��Ƶ
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                       \
  {                                                              \
    SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM); \
    SPI5->CR2 |= ((uint16_t)0x0003);                             \
    SPI5SetSpeedAndDataSize(0x0010, 0x0000);                     \
  }

#define MPU6500_SPI_DMA_Enable() \
  SPI5_DMA_Enable(DMA_RX_NUM)  // ��ʼһ��SPI��DMA����
//�궨��SPI��DMA�����жϺ����Լ������жϱ�־λ
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5

#define IMU_BOARD_INSTALL_SPIN_MATRIX \
  {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, { 0.0f, 0.0f, 1.0f }

//���ʹ��mpu6500������׼���ⲿ�жϣ�����ʹ������֪ͨ������������
static TaskHandle_t INSTask_Local_Handler;

// DMA��SPI ���͵�buf����INT_STATUS��ʼ������ȡ DMA_RX_NUM��С��ַ��ֵ
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] = {MPU_INT_STATUS |
                                                          MPU_SPI_READ_MSB};

static uint8_t
    mpu6500_spi_rxbuf[DMA_RX_NUM];  //������յ�ԭʼ����

//���������ǣ����ٶȼƣ����������ݵ����Զȣ���Ư
static void IMU_rotation_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3],
                               mpu6500_real_data_t *mpu6500,
                               ist8310_real_data_t *ist8310);

static mpu6500_real_data_t mpu6500_real_data;  //ת���ɹ��ʵ�λ��MPU6500����
static fp32 gyro_scale_factor[3][3] = {
    IMU_BOARD_INSTALL_SPIN_MATRIX};  //������У׼���Զ�
static fp32 gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};
static fp32 gyro_offset[3] = {0.0f, 0.0f, 0.0f};  //��������Ư
static fp32 accel_scale_factor[3][3] = {
    IMU_BOARD_INSTALL_SPIN_MATRIX};  //���ٶ�У׼���Զ�
static fp32 accel_offset[3] = {0.0f, 0.0f, 0.0f};  //���ٶ���Ư

static ist8310_real_data_t ist8310_real_data;  //ת���ɹ��ʵ�λ��IST8310����
static fp32 mag_scale_factor[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}};  //������У׼���Զ�
static fp32 mag_offset[3] = {0.0f, 0.0f, 0.0f};  //��������Ư

fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3];

void INSTask(void const *pvParameters) {
  //��ȡ��ǰ���������������������֪ͨ
  INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));

  osDelay(INS_TASK_INIT_TIME);
  //��ʼ��mpu6500��ʧ�ܽ�����ѭ��
  while (mpu6500_init() != MPU6500_NO_ERROR) {
    ;
  }

  //��ʼ��ist8310��ʧ�ܽ�����ѭ��
  while (ist8310_init() != IST8310_NO_ERROR) {
    ;
  }

  //��ʼ��SPI��DMA����ķ���
  MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);

  while (1) {
    //        //�ȴ��ⲿ�ж��жϻ�������
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
    }

    //����ȡ����mpu6500ԭʼ���ݴ����ɹ��ʵ�λ������
    mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET),
                      &mpu6500_real_data);

    //����ȡ����ist8310ԭʼ���ݴ����ɹ��ʵ�λ������
    ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET),
                      &ist8310_real_data);

    //��ȥ��Ư�Լ���ת����ϵ
    IMU_rotation_solve(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data,
                       &ist8310_real_data);

    //�ж��Ƿ���һ�ν��룬������һ������ʼ����Ԫ����֮��������Ԫ�������Ƕȵ�λrad
    static uint8_t updata_count = 0;
    if (mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT) {
      if (updata_count == 0) {
        //��ʼ����Ԫ��
        AHRS_init(INS_quat, INS_accel, INS_mag);
        get_angle(INS_quat, INS_angle, INS_angle + 1, INS_angle + 2);

        updata_count++;
      } else {
        //������Ԫ��
        AHRS_update(INS_quat, 0.001f, INS_gyro, INS_accel, INS_mag);
        get_angle(INS_quat, INS_angle, INS_angle + 1, INS_angle + 2);
      }  // update count if   code end
    }    // mpu6500 status  if end
    // while(1) end
  }
  // task function end
}

/**
 * @brief          У׼������
 * @author         RM
 * @param[in]
 * �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
 * @param[in]      �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
 * @param[in]      �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
 * @retval         ���ؿ�
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
 * @brief
 * У׼���������ã�����flash���������ط�����У׼ֵ
 * @author         RM
 * @param[in]
 * �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
 * @param[in]      �����ǵ���Ư
 * @retval         ���ؿ�
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
    //��������
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INSTask_Local_Handler, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
