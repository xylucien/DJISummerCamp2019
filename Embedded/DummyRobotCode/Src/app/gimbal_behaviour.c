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

#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "arm_math.h"
#include "bsp_buzzer.h"

#include "user_lib.h"

////云台校准蜂鸣器响声
//#define GIMBALWarnBuzzerOn() buzzer_on(31, 20000)
//#define GIMBALWarnBuzzerOFF() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
 * @brief 遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
 * @author         RM
 * @param[in]      输入的遥控器值
 * @param[in]      输出的死区处理后遥控器值
 * @param[in]      死区值
 * @retval         返回空
 */
#define rc_deadline_limit(input, output, dealine)                              \
  {                                                                            \
    if ((input) > (dealine) || (input) < -(dealine)) {                         \
      (output) = (input);                                                      \
    } else {                                                                   \
      (output) = 0;                                                            \
    }                                                                          \
  }

/**
 * @brief          云台校准的通过判断角速度来判断云台是否到达极限位置
 * @author         RM
 * @param[in]      对应轴的角速度，单位rad/s
 * @param[in]      计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
 * @param[in]      记录的角度 rad
 * @param[in]      反馈的角度 rad
 * @param[in]      记录的编码值 raw
 * @param[in]      反馈的编码值 raw
 * @param[in]      校准的步骤 完成一次 加一
 * @retval         返回空
 */
#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, \
                               step)                                           \
  {                                                                            \
    if ((gyro) < GIMBAL_CALI_GYRO_LIMIT) {                                     \
      (cmd_time)++;                                                            \
      if ((cmd_time) > GIMBAL_CALI_STEP_TIME) {                                \
        (cmd_time) = 0;                                                        \
        (angle_set) = (angle);                                                 \
        (ecd_set) = (ecd);                                                     \
        (step)++;                                                              \
      }                                                                        \
    }                                                                          \
  }

/**
 * @brief 云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
 * @author         RM
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          云台无力控制，在这个模式下发送的yaw，pitch
 * 是电机控制原始值，云台电机发送can零控制量，使得云台无力
 * @author         RM
 * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch,
                                      gimbal_control_t *gimbal_control_set);

/**
 * @brief
 * 云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
 * @author         RM
 * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch,
                                gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch,
                                          gimbal_control_t *gimbal_control_set);

//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
 * @brief          云台行为状态机以及电机状态机设置
 * @author         RM
 * @param[in]      云台数据指针
 * @retval         返回空
 */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set) {
  if (gimbal_mode_set == NULL) {
    return;
  }
  //云台行为状态机设置
  gimbal_behavour_set(gimbal_mode_set);

  //根据云台行为状态机设置电机状态机
  if (gimbal_behaviour == GIMBAL_ZERO_FORCE) {
    gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  } else if (gimbal_behaviour == GIMBAL_CALI) {
    gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  } else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE) {
    gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode =
        GIMBAL_MOTOR_ENCONDE;
  }
}

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @author         RM
 * @param[in]      设置的yaw角度增加值，单位 rad
 * @param[in]      设置的pitch角度增加值，单位 rad
 * @param[in]      云台数据指针
 * @retval         返回空
 */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch,
                                  gimbal_control_t *gimbal_control_set) {

  if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL) {
    return;
  }

  static fp32 rc_add_yaw, rc_add_pit;
  static int16_t yaw_channel = 0, pitch_channel = 0;

  //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
  rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL],
                    yaw_channel, RC_DEADBAND);
  rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL],
                    pitch_channel, RC_DEADBAND);

  rc_add_yaw = yaw_channel * YAW_RC_SEN -
               gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
  rc_add_pit = pitch_channel * PITCH_RC_SEN +
               gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;

  if (gimbal_behaviour == GIMBAL_ZERO_FORCE) {
    gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
  } else if (gimbal_behaviour == GIMBAL_CALI) {
    gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
  } else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE) {
    gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
  }
  //将控制增加量赋值
  *add_yaw = rc_add_yaw;
  *add_pitch = rc_add_pit;
}

/**
 * @brief 云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
 * @author         RM
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set) {
  if (gimbal_mode_set == NULL) {
    return;
  }

  if (toe_is_error(DBUSTOE)) {
    gimbal_behaviour = GIMBAL_ZERO_FORCE;
    return;
  }
  //校准行为，return 不会设置其他的模式
  if (gimbal_behaviour == GIMBAL_CALI &&
      gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP &&
      gimbal_mode_set->gimbal_cali.step != 0) {
    return;
  }
  //如果外部使得校准步骤从0 变成 start，则进入校准模式
  if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP &&
      !toe_is_error(DBUSTOE)) {
    gimbal_behaviour = GIMBAL_CALI;
    return;
  }

  //开关控制 云台状态
  if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL])) {
    gimbal_behaviour = GIMBAL_ZERO_FORCE;
  } else if (switch_is_mid(
                 gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL])) {
    gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
  } else if (switch_is_up(
                 gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL])) {
    gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
  }
}

/**
 * @brief          云台无力控制，在这个模式下发送的yaw，pitch
 * 是电机控制原始值，云台电机发送can零控制量，使得云台无力
 * @author         RM
 * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch,
                                      gimbal_control_t *gimbal_control_set) {
  if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
    return;
  }

  *yaw = 0.0f;
  *pitch = 0.0f;
}

/**
 * @brief
 * 云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
 * @author         RM
 * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch,
                                gimbal_control_t *gimbal_control_set) {
  if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
    return;
  }
  static uint16_t cali_time = 0;

  if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP) {

    *pitch = GIMBAL_CALI_MOTOR_SET;
    *yaw = 0;

    //判断陀螺仪数据， 并记录最大最小角度数据
    GIMBAL_CALI_GYRO_JUDGE(
        gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time,
        gimbal_control_set->gimbal_cali.max_pitch,
        gimbal_control_set->gimbal_pitch_motor.relative_angle,
        gimbal_control_set->gimbal_cali.max_pitch_ecd,
        gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd,
        gimbal_control_set->gimbal_cali.step);
  } else if (gimbal_control_set->gimbal_cali.step ==
             GIMBAL_CALI_PITCH_MIN_STEP) {
    *pitch = -GIMBAL_CALI_MOTOR_SET;
    *yaw = 0;

    GIMBAL_CALI_GYRO_JUDGE(
        gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time,
        gimbal_control_set->gimbal_cali.min_pitch,
        gimbal_control_set->gimbal_pitch_motor.relative_angle,
        gimbal_control_set->gimbal_cali.min_pitch_ecd,
        gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd,
        gimbal_control_set->gimbal_cali.step);
  } else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP) {
    *pitch = 0;
    *yaw = GIMBAL_CALI_MOTOR_SET;

    GIMBAL_CALI_GYRO_JUDGE(
        gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time,
        gimbal_control_set->gimbal_cali.max_yaw,
        gimbal_control_set->gimbal_yaw_motor.relative_angle,
        gimbal_control_set->gimbal_cali.max_yaw_ecd,
        gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd,
        gimbal_control_set->gimbal_cali.step);
  }

  else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP) {
    *pitch = 0;
    *yaw = -GIMBAL_CALI_MOTOR_SET;

    GIMBAL_CALI_GYRO_JUDGE(
        gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time,
        gimbal_control_set->gimbal_cali.min_yaw,
        gimbal_control_set->gimbal_yaw_motor.relative_angle,
        gimbal_control_set->gimbal_cali.min_yaw_ecd,
        gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd,
        gimbal_control_set->gimbal_cali.step);
  } else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP) {
    cali_time = 0;
  }
}

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void
gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch,
                              gimbal_control_t *gimbal_control_set) {
  if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
    return;
  }
  //不需要处理，
}
