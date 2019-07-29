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

#include "gimbal_task.h"

#include "main.h"

#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "INS_Task.h"
#include "arm_math.h"
#include "bsp_power_ctrl.h"
#include "gimbal_behaviour.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

#include "cmsis_os.h"

//电机编码值规整 0—8191
#define ECD_FORMAT(ecd)                                                        \
  {                                                                            \
    if ((ecd) > ECD_RANGE)                                                     \
      (ecd) -= ECD_RANGE;                                                      \
    else if ((ecd) < 0)                                                        \
      (ecd) += ECD_RANGE;                                                      \
  }

extern gimbal_control_t gimbal_control;

//发送的can 指令
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;

extern void gimbal_pid_init(void);
extern void J_scope_show(void);
extern void yaw_motor_relative_angle_control(gimbal_control_t *gimbal_motor);
extern void pitch_motor_relative_angle_control(gimbal_control_t *gimbal_motor);

//云台初始化
static void gimbal_init(gimbal_control_t *gimbal_init);

//云台状态设置
static void gimbal_set_mode(gimbal_control_t *set_mode);

//云台数据更新
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

//云台状态切换保存数据，例如从陀螺仪状态切换到编码器状态保存目标值
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

//设置云台控制量
static void gimbal_set_control(gimbal_control_t *set_control);

//云台控制pid计算
static void gimbal_control_loop(gimbal_control_t *control_loop);

//计算云台电机相对中值的相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

//在陀螺仪角度控制下，对控制的目标值进限制以防超最大相对角度
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

static void calc_gimbal_cali(const gimbal_control_cali_t *gimbal_cali,
                             uint16_t *yaw_offset, uint16_t *pitch_offset,
                             fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch,
                             fp32 *min_pitch);

void gimbal_task(void const *pvParameters) {
  for (uint8_t i = 0; i < 4; i++) {
    power_ctrl_on(i);
    osDelay(GIMBAL_TASK_INIT_TIME);
  }

  //云台初始化
  gimbal_init(&gimbal_control);

  //判断电机是否都上线
  while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE)) {
    osDelay(GIMBAL_CONTROL_TIME);
    gimbal_feedback_update(&gimbal_control); //云台数据反馈
  }

  while (1) {
    gimbal_set_mode(&gimbal_control); //设置云台控制模式
    gimbal_mode_change_control_transit(
        &gimbal_control);                    //控制模式切换 控制数据过渡
    gimbal_feedback_update(&gimbal_control); //云台数据反馈
    gimbal_set_control(&gimbal_control);     //设置云台控制量
    gimbal_control_loop(&gimbal_control);    //云台控制PID计算
#if YAW_TURN
    yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
    yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
    pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
    pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

    //云台在遥控器掉线状态即relax
    //状态，can指令为0，不使用current设置为零的方法，是保证遥控器掉线一定使得云台停止
    if (!(toe_is_error(YawGimbalMotorTOE) &&
          toe_is_error(PitchGimbalMotorTOE))) {
      if (toe_is_error(DBUSTOE)) {
        CAN_CMD_GIMBAL(0, 0, 0, 0);
      } else {
        CAN_CMD_GIMBAL(yaw_can_set_current, pitch_can_set_current, 0, 0);
      }
    }

#if GIMBAL_TEST_MODE
    J_scope_show();
#endif

    osDelay(GIMBAL_CONTROL_TIME);
  }
}

//初始化pid 数据指针
static void gimbal_init(gimbal_control_t *init) {
  //电机数据指针获取
  init->gimbal_yaw_motor.gimbal_motor_measure =
      get_Yaw_Gimbal_Motor_Measure_Point();
  init->gimbal_pitch_motor.gimbal_motor_measure =
      get_Pitch_Gimbal_Motor_Measure_Point();
  //陀螺仪数据指针获取

  init->gimbal_INS_gyro_point = get_MPU6500_gyro_data_point();
  //遥控器数据指针获取
  init->gimbal_rc_ctrl = get_remote_control_point();
  //初始化电机模式
  init->gimbal_yaw_motor.gimbal_motor_mode =
      init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  init->gimbal_pitch_motor.gimbal_motor_mode =
      init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  //初始化yaw电机pid

  gimbal_feedback_update(init);

  init->gimbal_yaw_motor.relative_angle_set =
      init->gimbal_yaw_motor.relative_angle;
  init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

  init->gimbal_pitch_motor.relative_angle_set =
      init->gimbal_pitch_motor.relative_angle;
  init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

  gimbal_pid_init();
}

static void gimbal_set_mode(gimbal_control_t *set_mode) {
  if (set_mode == NULL) {
    return;
  }
  gimbal_behaviour_mode_set(set_mode);
}

static void gimbal_feedback_update(gimbal_control_t *feedback_update) {
  if (feedback_update == NULL) {
    return;
  }
  //云台数据更新
  feedback_update->gimbal_pitch_motor.relative_angle =
      motor_ecd_to_angle_change(
          feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
          feedback_update->gimbal_pitch_motor.offset_ecd);
  feedback_update->gimbal_pitch_motor.motor_gyro =
      *(feedback_update->gimbal_INS_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

  feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(
      feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
      feedback_update->gimbal_yaw_motor.offset_ecd);

  feedback_update->gimbal_yaw_motor.motor_gyro =
      arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) *
          (*(feedback_update->gimbal_INS_gyro_point +
             INS_GYRO_Z_ADDRESS_OFFSET)) -
      arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) *
          (*(feedback_update->gimbal_INS_gyro_point +
             INS_GYRO_X_ADDRESS_OFFSET));
}
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd) {
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > HALF_ECD_RANGE) {
    relative_ecd -= ECD_RANGE;
  } else if (relative_ecd < -HALF_ECD_RANGE) {
    relative_ecd += ECD_RANGE;
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

//云台状态切换保存，用于状态切换过渡
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change) {
  if (mode_change == NULL) {
    return;
  }
  // yaw电机状态机切换保存数据
  if (mode_change->gimbal_yaw_motor.last_gimbal_motor_mode !=
          GIMBAL_MOTOR_RAW &&
      mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
    mode_change->gimbal_yaw_motor.raw_cmd_current =
        mode_change->gimbal_yaw_motor.current_set =
            mode_change->gimbal_yaw_motor.given_current;
  } else if (mode_change->gimbal_yaw_motor.last_gimbal_motor_mode !=
                 GIMBAL_MOTOR_ENCONDE &&
             mode_change->gimbal_yaw_motor.gimbal_motor_mode ==
                 GIMBAL_MOTOR_ENCONDE) {
    mode_change->gimbal_yaw_motor.relative_angle_set =
        mode_change->gimbal_yaw_motor.relative_angle;
  }
  mode_change->gimbal_yaw_motor.last_gimbal_motor_mode =
      mode_change->gimbal_yaw_motor.gimbal_motor_mode;

  // pitch电机状态机切换保存数据
  if (mode_change->gimbal_pitch_motor.last_gimbal_motor_mode !=
          GIMBAL_MOTOR_RAW &&
      mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
    mode_change->gimbal_pitch_motor.raw_cmd_current =
        mode_change->gimbal_pitch_motor.current_set =
            mode_change->gimbal_pitch_motor.given_current;
  } else if (mode_change->gimbal_pitch_motor.last_gimbal_motor_mode !=
                 GIMBAL_MOTOR_ENCONDE &&
             mode_change->gimbal_pitch_motor.gimbal_motor_mode ==
                 GIMBAL_MOTOR_ENCONDE) {
    mode_change->gimbal_pitch_motor.relative_angle_set =
        mode_change->gimbal_pitch_motor.relative_angle;
  }

  mode_change->gimbal_pitch_motor.last_gimbal_motor_mode =
      mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

//云台控制量设置
static void gimbal_set_control(gimbal_control_t *set_control) {
  if (set_control == NULL) {
    return;
  }

  fp32 add_yaw_angle = 0.0f;
  fp32 add_pitch_angle = 0.0f;

  gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
  // yaw电机模式控制
  if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
    // raw模式下，直接发送控制值
    set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
  } else if (set_control->gimbal_yaw_motor.gimbal_motor_mode ==
             GIMBAL_MOTOR_ENCONDE) {
    // enconde模式下，电机编码角度控制
    gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
  }

  // pitch电机模式控制
  if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
    // raw模式下，直接发送控制值
    set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
  } else if (set_control->gimbal_pitch_motor.gimbal_motor_mode ==
             GIMBAL_MOTOR_ENCONDE) {
    // enconde模式下，电机编码角度控制
    gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor,
                                add_pitch_angle);
  }
}

static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor,
                                        fp32 add) {
  if (gimbal_motor == NULL) {
    return;
  }
  gimbal_motor->relative_angle_set += add;
  //是否超过最大 最小值
  if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle) {
    gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
  } else if (gimbal_motor->relative_angle_set <
             gimbal_motor->min_relative_angle) {
    gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
  }
}
//云台控制状态使用不同控制pid
static void gimbal_control_loop(gimbal_control_t *control_loop) {
  if (control_loop == NULL) {
    return;
  }
  // yaw不同模式对于不同的控制函数
  if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
    // raw控制
    gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
  } else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode ==
             GIMBAL_MOTOR_ENCONDE) {
    // enconde角度控制
    yaw_motor_relative_angle_control(control_loop);
  }

  // pitch不同模式对于不同的控制函数
  if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
    // raw控制
    gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
  } else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode ==
             GIMBAL_MOTOR_ENCONDE) {
    // enconde角度控制
    pitch_motor_relative_angle_control(control_loop);
  }
}

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor) {
  if (gimbal_motor == NULL) {
    return;
  }
  gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @author         RM
 * @param[in]      yaw 中值
 * @param[in]      pitch 中值
 * @param[in]      yaw 最大相对角度
 * @param[in]      yaw 最小相对角度
 * @param[in]      pitch 最大相对角度
 * @param[in]      pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control
 * 静态变量导致函数不适用以上通用指针复用
 */
void set_cali_gimbal_hook(const uint16_t yaw_offset,
                          const uint16_t pitch_offset, const fp32 max_yaw,
                          const fp32 min_yaw, const fp32 max_pitch,
                          const fp32 min_pitch) {
  gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

  gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
 * @brief          云台校准计算，将校准记录的最大 最小值 来计算云台
 * 中值和最大最小机械角度
 * @author         RM
 * @param[in]      yaw 中值 指针
 * @param[in]      pitch 中值 指针
 * @param[in]      yaw 最大相对角度 指针
 * @param[in]      yaw 最小相对角度 指针
 * @param[in]      pitch 最大相对角度 指针
 * @param[in]      pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control
 * 静态变量导致函数不适用以上通用指针复用
 */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset,
                            fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch,
                            fp32 *min_pitch) {
  if (gimbal_control.gimbal_cali.step == 0) {
    gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
    //保存进入时候的数据，作为起始数据，来判断最大，最小值
    gimbal_control.gimbal_cali.max_pitch =
        gimbal_control.gimbal_pitch_motor.relative_angle;
    gimbal_control.gimbal_cali.max_pitch_ecd =
        gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.max_yaw =
        gimbal_control.gimbal_yaw_motor.relative_angle;
    gimbal_control.gimbal_cali.max_yaw_ecd =
        gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_pitch =
        gimbal_control.gimbal_pitch_motor.relative_angle;
    gimbal_control.gimbal_cali.min_pitch_ecd =
        gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_yaw =
        gimbal_control.gimbal_yaw_motor.relative_angle;
    gimbal_control.gimbal_cali.min_yaw_ecd =
        gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    return 0;
  } else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP) {
    calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset,
                     max_yaw, min_yaw, max_pitch, min_pitch);
    (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
    (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
    gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
    gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
    gimbal_control.gimbal_cali.step = 0;
    return 1;
  } else {
    return 0;
  }
}

//校准计算，相对最大角度，云台中值
static void calc_gimbal_cali(const gimbal_control_cali_t *gimbal_cali,
                             uint16_t *yaw_offset, uint16_t *pitch_offset,
                             fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch,
                             fp32 *min_pitch) {
  if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL ||
      max_yaw == NULL || min_yaw == NULL || max_pitch == NULL ||
      min_pitch == NULL) {
    return;
  }

  int16_t temp_ecd = 0;

#if YAW_TURN

#else

  temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

  if (temp_ecd < 0) {
    temp_ecd += ECD_RANGE;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

  ECD_FORMAT(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

#else

  temp_ecd = gimbal_cali->max_pitch_ecd - gimbal_cali->min_pitch_ecd;

  if (temp_ecd < 0) {
    temp_ecd += ECD_RANGE;
  }
  temp_ecd = gimbal_cali->max_pitch_ecd - (temp_ecd / 2);

  ECD_FORMAT(temp_ecd);
  *pitch_offset = temp_ecd;
  *max_pitch =
      motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch =
      motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#endif
}
