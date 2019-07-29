/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief 完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  * 故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  * 状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  * 状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
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

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 37
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define MODE_CHANNEL 0

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

// yaw，pitch角度与遥控器输入比例
#define YAW_RC_SEN -0.000005f
#define PITCH_RC_SEN -0.000006f // 0.005

// yaw,pitch角度和鼠标输入的比例
#define YAW_MOUSE_SEN 0.00005f
#define PITCH_MOUSE_SEN 0.00015f

//云台控制周期
#define GIMBAL_CONTROL_TIME 1

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 1

//电机是否反装
#define PITCH_TURN 0
#define YAW_TURN 0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

#define GIMBAL_CALI_REDUNDANT_ANGLE 0.01f

typedef enum {
  GIMBAL_MOTOR_RAW = 0, //电机原始值控制
  GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct {
  const motor_measure_t *gimbal_motor_measure;
  PidTypeDef gimbal_motor_angle_pid;
  PidTypeDef gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  uint16_t offset_ecd;
  fp32 max_relative_angle; // rad
  fp32 min_relative_angle; // rad

  fp32 relative_angle;     // rad
  fp32 relative_angle_set; // rad

  fp32 motor_gyro; // rad/s
  fp32 motor_gyro_set;
  fp32 motor_speed;
  fp32 raw_cmd_current;
  fp32 current_set;
  int16_t given_current;

} gimbal_motor_t;

typedef struct {
  fp32 max_yaw;
  fp32 min_yaw;
  fp32 max_pitch;
  fp32 min_pitch;
  uint16_t max_yaw_ecd;
  uint16_t min_yaw_ecd;
  uint16_t max_pitch_ecd;
  uint16_t min_pitch_ecd;
  uint8_t step;
} gimbal_control_cali_t;

typedef struct {
  const RC_ctrl_t *gimbal_rc_ctrl;
  const fp32 *gimbal_INS_angle_point;
  const fp32 *gimbal_INS_gyro_point;
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
  gimbal_control_cali_t gimbal_cali;
} gimbal_control_t;

extern void gimbal_task(void const *pvParameters);

extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset,
                                   fp32 *max_yaw, fp32 *min_yaw,
                                   fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset,
                                 const uint16_t pitch_offset,
                                 const fp32 max_yaw, const fp32 min_yaw,
                                 const fp32 max_pitch, const fp32 min_pitch);

#endif
