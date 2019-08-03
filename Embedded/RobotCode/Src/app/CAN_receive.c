/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
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

#include "CAN_receive.h"

#include "bsp_rng.h"
#include "stm32f4xx.h"

#include "Detect_Task.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "string.h"
#include "task.h"

#include "cmsis_os.h"
#include <stdbool.h>

#include "MathUtils.h"
#include "CANMessage.h"
#include "CANUtil.h"

extern CAN_HandleTypeDef hcan1;

extern QueueHandle_t canTargetVelocityQueue;

rm_imu_data_t rm_imu_data;
Twist2D targetVelocity;

float lastVx, lastVy, lastVw;

static int16_t motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

#define get_motor_measure(ptr, data)                                           \
  {                                                                            \
    (ptr)->last_ecd = (ptr)->ecd;                                              \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                       \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);                 \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);             \
    (ptr)->temperate = (data)[6];                                              \
    (ptr)->delta_ecd = motor_ecd_to_angle_change((ptr)->ecd, (ptr)->last_ecd); \
    (ptr)->total_ecd += (ptr)->delta_ecd;                                      \
  }

static motor_measure_t motor_chassis[7];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
	
float test = 0.0;
CAN_RxHeaderTypeDef rx_header;
	
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId) {
  case CAN_3508_M1_ID:
  case CAN_3508_M2_ID:
  case CAN_3508_M3_ID:
  case CAN_3508_M4_ID:
  case CAN_YAW_MOTOR_ID:
  case CAN_PIT_MOTOR_ID:
  case CAN_TRIGGER_MOTOR_ID: {
    static uint8_t i = 0;
    //�������ID��
    i = rx_header.StdId - CAN_3508_M1_ID;
    //����������ݺ꺯��
    get_motor_measure(&motor_chassis[i], rx_data);
    //��¼ʱ��
    DetectHook(ChassisMotor1TOE + i);
    break;
  }

  case RM_IMU_PARAM_ID: {
    rm_imu_data.accel_rangle = rx_data[0] & 0x0F;
    rm_imu_data.gyro_rangle = (rx_data[0] & 0xF0) >> 4;
    rm_imu_data.sensor_control_temperature = rx_data[2];
    rm_imu_data.imu_sensor_rotation = rx_data[3] & 0x1F;
    rm_imu_data.ahrs_rotation_sequence = (rx_data[3] & 0xE0) >> 5;
    rm_imu_data.quat_euler = rx_data[4] & 0x01;

    switch (rm_imu_data.gyro_rangle) {
    case 0:
      rm_imu_data.gyro_sen = GYRO_2000_SEN;
      break;
    case 1:
      rm_imu_data.gyro_sen = GYRO_1000_SEN;
      break;
    case 2:
      rm_imu_data.gyro_sen = GYRO_500_SEN;
      break;
    case 3:
      rm_imu_data.gyro_sen = GYRO_250_SEN;
      break;
    case 4:
      rm_imu_data.gyro_sen = GYRO_125_SEN;
      break;
    }

    switch (rm_imu_data.accel_rangle) {
    case 0:
      rm_imu_data.accel_sen = ACCEL_3G_SEN;
      break;
    case 1:
      rm_imu_data.accel_sen = ACCEL_6G_SEN;
      break;
    case 2:
      rm_imu_data.accel_sen = ACCEL_12G_SEN;
      break;
    case 3:
      rm_imu_data.accel_sen = ACCEL_24G_SEN;
      break;
    }

    break;
  }

  case RM_IMU_QUAT_ID: {
    if (rm_imu_data.quat_euler && rx_header.DLC == 6) {
      memcpy(rm_imu_data.euler_angle, rx_data, rx_header.DLC);
      rm_imu_data.euler_angle_fp32[0] = rm_imu_data.euler_angle[0] * 0.0001f;
      rm_imu_data.euler_angle_fp32[1] = rm_imu_data.euler_angle[1] * 0.0001f;
      rm_imu_data.euler_angle_fp32[2] = rm_imu_data.euler_angle[2] * 0.0001f;
    } else if (rm_imu_data.quat_euler == 0 && rx_header.DLC == 8) {
      memcpy(rm_imu_data.quat, rx_data, rx_header.DLC);
      rm_imu_data.quat_fp32[0] = rm_imu_data.quat[0] * 0.0001f;
      rm_imu_data.quat_fp32[1] = rm_imu_data.quat[1] * 0.0001f;
      rm_imu_data.quat_fp32[2] = rm_imu_data.quat[2] * 0.0001f;
      rm_imu_data.quat_fp32[3] = rm_imu_data.quat[3] * 0.0001f;
    }

    break;
  }

  case RM_IMU_GYRO_ID: {
    memcpy(rm_imu_data.gyro_int16, rx_data, 6);
    rm_imu_data.gyro_fp32[0] = rm_imu_data.gyro_int16[0] * rm_imu_data.gyro_sen;
    rm_imu_data.gyro_fp32[1] = rm_imu_data.gyro_int16[1] * rm_imu_data.gyro_sen;
    rm_imu_data.gyro_fp32[2] = rm_imu_data.gyro_int16[2] * rm_imu_data.gyro_sen;
    rm_imu_data.sensor_temperature =
        (int16_t)((rx_data[6] << 3) | (rx_data[7] >> 5));
    if (rm_imu_data.sensor_temperature > 1023) {
      rm_imu_data.sensor_temperature -= 2048;
    }
    break;
  }

  case RM_IMU_ACCEL_ID: {
    memcpy(rm_imu_data.accel_int16, rx_data, 6);
    rm_imu_data.accel_fp32[0] =
        rm_imu_data.accel_int16[0] * rm_imu_data.accel_sen;
    rm_imu_data.accel_fp32[1] =
        rm_imu_data.accel_int16[1] * rm_imu_data.accel_sen;
    rm_imu_data.accel_fp32[2] =
        rm_imu_data.accel_int16[2] * rm_imu_data.accel_sen;
    memcpy(&rm_imu_data.sensor_time, (rx_data + 6), 2);
    break;
  }

  case RM_IMU_MAG_ID: {
    memcpy(rm_imu_data.mag_int16, rx_data, 6);
    break;
  }

  case CAN_MANIFOLD_ID - 1: {
    uint8_t messageId = deserializeInt(rx_data);

    switch(messageId){
      case CANMESSAGE_ID_TEST: {
        float messageTest = deserializeFloat(rx_data + 4);
				test = messageTest;
        break;
      }

      case CANMESSAGE_ID_TARGET_VX: {
        float vX = deserializeFloat(rx_data + 4);
        lastVx = vX;
				//xQueueSend(canTargetVXQueue, &vX, (TickType_t)0);
        break;
      }

      case CANMESSAGE_ID_TARGET_VY: {
        float vY = deserializeFloat(rx_data + 4);
        lastVy = vY;
				//xQueueSend(canTargetVYQueue, &vY, (TickType_t)0);
        break;
      }

      case CANMESSAGE_ID_TARGET_VW: {
        float vW = deserializeFloat(rx_data + 4);
        lastVw = vW;
				//xQueueSend(canTargetVWQueue, &vW, (TickType_t)0);
        break;
      }

      case CANMESSAGE_ID_TARGET_READY: {
        //portENTER_CRITICAL();
        targetVelocity.vX = lastVx;
        targetVelocity.vY = lastVy;
        targetVelocity.w = lastVw;
        //portENTER_CRITICAL();
				
        break;
      }

      default: {
				
        break;
      }
    }

    break;
  }

  default: {
    break;
  }
  }
}

// CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
void CAN_CMD_CHASSIS_RESET_ID(void) {
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data,
                       &send_mail_box);
}

//���͵��̵����������
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3,
                     int16_t motor4) {
  uint32_t send_mail_box;
											 
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;

  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data,
                       &send_mail_box);
}

const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void) {
  return &motor_chassis[4];
}

const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void) {
  return &motor_chassis[5];
}

const motor_measure_t *get_Trigger_Motor_Measure_Point(void) {
  return &motor_chassis[6];
}

const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i) {
  return &motor_chassis[(i & 0x03)];
}

static int16_t motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd) {
  int16_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > HALF_ECD_RANGE) {
    relative_ecd -= ECD_RANGE;
  } else if (relative_ecd < -HALF_ECD_RANGE) {
    relative_ecd += ECD_RANGE;
  }

  return relative_ecd;
}
