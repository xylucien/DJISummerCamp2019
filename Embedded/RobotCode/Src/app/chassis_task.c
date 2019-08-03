/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
  * @note
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
#include "chassis_task.h"

#include "remote_control.h"

#include "cmsis_os.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "INS_Task.h"
#include "Remote_Control.h"

#include "chassis_behaviour.h"
#include "MathUtils.h"

#define rc_deadline_limit(input, output, dealine)                              \
  {                                                                            \
    if ((input) > (dealine) || (input) < -(dealine)) {                         \
      (output) = (input);                                                      \
    } else {                                                                   \
      (output) = 0;                                                            \
    }                                                                          \
  }

extern chassis_move_t chassis_move;

extern void chassis_motor_speed_update(chassis_move_t *chassis_move_update);
extern void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set,
                                                  const fp32 vy_set,
                                                  const fp32 wz_set,
                                                  fp32 wheel_speed[4]);

extern void chassis_PID_init(void);

//���̳�ʼ������Ҫ��pid��ʼ��
static void chassis_init(chassis_move_t *chassis_move_init);
//����״̬��ѡ��ͨ��ң�����Ŀ���
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//�������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����״̬�ı�����������ĸı�static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//�������ø���ң����������
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//����PID�����Լ��˶��ֽ�
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//������
void chassis_task(void const *pvParameters) {
  //����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //���̳�ʼ��
  chassis_init(&chassis_move);
  //�жϵ��̵���Ƿ�����
  while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) ||
         toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) ||
         toe_is_error(DBUSTOE)) {
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }

  while (1) {
    //ң��������״̬
    chassis_set_mode(&chassis_move);
    //ң����״̬�л����ݱ���
    chassis_mode_change_control_transit(&chassis_move);
    //�������ݸ���
    chassis_feedback_update(&chassis_move);
    //���̿���������
    chassis_set_contorl(&chassis_move);
    //���̿���PID����
    chassis_control_loop(&chassis_move);

    if (!(toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) ||
          toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE))) {
      //��ң�������ߵ�ʱ��Ϊrelax״̬�����̵��ָ��Ϊ�㣬Ϊ�˱�֤һ������Ϊ�㣬�ʶ�����������give_current�ķ���
      if (toe_is_error(DBUSTOE)) {
        CAN_CMD_CHASSIS(0, 0, 0, 0);
      } else {
        CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current,
                        chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current,
                        chassis_move.motor_chassis[3].give_current);
      }
    }
    //ϵͳ��ʱ
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}

static void chassis_init(chassis_move_t *chassis_move_init) {
  if (chassis_move_init == NULL) {
    return;
  }

  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

  uint8_t i;

  //���̿���״̬Ϊֹͣ
  chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
  //��ȡң����ָ��
  chassis_move_init->chassis_RC = get_remote_control_point();
  //��ȡ��������̬��ָ��
  // chassis_move_init->chassis_INS_angle = get_INS_angle_point();
  //��ȡ������
  chassis_move_init->chassis_INS_gyro = get_MPU6500_gyro_data_point();

  //��ʼ��PID �˶�
  for (i = 0; i < 4; i++) {
    chassis_move.motor_chassis[i].chassis_motor_measure =
        get_Chassis_Motor_Measure_Point(i);
  }

  //��һ���˲�����б����������
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx,
                          CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy,
                          CHASSIS_CONTROL_TIME, chassis_y_order_filter);

  //��� ��С�ٶ�
  chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  chassis_PID_init();

  //����һ������
  chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode) {
  if (chassis_move_mode == NULL) {
    return;
  }

  chassis_behaviour_mode_set(chassis_move_mode);
}

static void
chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit) {
  if (chassis_move_transit == NULL) {
    return;
  }

  if (chassis_move_transit->last_chassis_mode ==
      chassis_move_transit->chassis_mode) {
    return;
  }

  //���������̽Ƕ�ģʽ
  if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPEED) &&
      chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPEED) {
    chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }
  //���벻������̨ģʽ
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_RAW) &&
           chassis_move_transit->chassis_mode == CHASSIS_VECTOR_RAW) {
    chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }

  chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
static void chassis_feedback_update(chassis_move_t *chassis_move_update) {
  if (chassis_move_update == NULL) {
    return;
  }
  chassis_motor_speed_update(chassis_move_update);

  //���������̬�Ƕ�, �����������������������ⲿ�ִ���
  chassis_move_update->chassis_yaw = rad_format(
      *(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
  chassis_move_update->chassis_pitch = rad_format(
      *(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
  chassis_move_update->chassis_roll =
      *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

  chassis_move_update->chassis_gyro_x =
      *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);
  chassis_move_update->chassis_gyro_y =
      *(chassis_move_update->chassis_INS_gyro + INS_GYRO_Y_ADDRESS_OFFSET);
  chassis_move_update->chassis_gyro_z =
      *(chassis_move_update->chassis_INS_gyro + INS_GYRO_Z_ADDRESS_OFFSET);
}

//ң���������ݴ����ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set,
                                  chassis_move_t *chassis_move_rc_to_vector) {
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) {
    return;
  }
  //ң����ԭʼͨ��ֵ
  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;
  //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
  rc_deadline_limit(
      chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL],
      vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadline_limit(
      chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL],
      vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY) {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
  } else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY) {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY) {
    vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
  } else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY) {
    vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
  }

  //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx,
                          vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy,
                          vy_set_channel);

  //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN &&
      vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN &&
      vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN) {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

//����ң�������������
static void chassis_set_contorl(chassis_move_t *chassis_move_control) {

  if (chassis_move_control == NULL) {
    return;
  }

  //�����ٶ�
  fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;
  chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set,
                                chassis_move_control);

  if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPEED) {
    //���õ����˶����ٶ�
    chassis_move_control->vx_set =
        fp32_constrain(vx_set, chassis_move_control->vx_min_speed,
                       chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set =
        fp32_constrain(vy_set, chassis_move_control->vy_min_speed,
                       chassis_move_control->vy_max_speed);
    //������ת�Ľ��ٶ�
    chassis_move_control->wz_set = wz_set;
  } else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW) {
    chassis_move_control->vx_set = vx_set;
    chassis_move_control->vy_set = vy_set;
    chassis_move_control->wz_set = wz_set;
    chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
  }
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop) {
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;
  //�����˶��ֽ�
  chassis_vector_to_mecanum_wheel_speed(
      chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set,
      chassis_move_control_loop->wz_set, wheel_speed);

  if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW) {
    //��ֵ����ֵ
    for (i = 0; i < 4; i++) {
      chassis_move_control_loop->motor_chassis[i].give_current =
          (int16_t)(wheel_speed[i]);
    }
    // raw����ֱ�ӷ���
    return;
  }

  //�������ӿ�������ٶȣ�������������ٶ�
  for (i = 0; i < 4; i++) {
    chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
    temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
    if (max_vector < temp) {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED) {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++) {
      chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
  }

  //����pid

  for (i = 0; i < 4; i++) {
    PID_Calc(&chassis_move_control_loop->motor_speed_pid[i],
             chassis_move_control_loop->motor_chassis[i].speed,
             chassis_move_control_loop->motor_chassis[i].speed_set);
  }

  //��ֵ����ֵ
  for (i = 0; i < 4; i++) {
    chassis_move_control_loop->motor_chassis[i].give_current =
        (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
  }
}
