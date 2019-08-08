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
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "CAN_Receive.h"
#include "struct_typedef.h"

#include "Remote_Control.h"
#include "pid.h"
#include "user_lib.h"
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.0015f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.0015f

#define CHASSIS_WZ_RC_SEN 0.0005f

#define CHASSIS_ACCEL_X_NUM 0.1066666667f
#define CHASSIS_ACCEL_Y_NUM 0.2333333333f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.25f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 1
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.001f
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 1000.0f
//����c610 ���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 10000.0f

//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

// m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

//���̵������ٶ�
#define MAX_WHEEL_SPEED 1.5f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.25f
//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ
//0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.1f

typedef enum {
  CHASSIS_VECTOR_SPEED,
  CHASSIS_VECTOR_RAW,
} chassis_mode_e;

typedef struct {
  const motor_measure_t *chassis_motor_measure;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct {
  const RC_ctrl_t *chassis_RC;   //����ʹ�õ�ң����ָ��
  const fp32 *chassis_INS_angle; //��ȡ�����ǽ������ŷ����ָ��
  const fp32 *chassis_INS_gyro;
  chassis_mode_e chassis_mode;      //���̿���״̬��
  chassis_mode_e last_chassis_mode; //�����ϴο���״̬��
  chassis_motor_t motor_chassis[4]; //���̵������
  PidTypeDef motor_speed_pid[4];    //���̵���ٶ�pid
  PidTypeDef chassis_rotation_pid;  //������ת�ٶ�pid
  PidTypeDef chassis_angle_pid;     //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;     //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;     //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;     //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set; //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set; //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set; //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

  fp32 chassis_yaw_set;

  fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
  fp32 chassis_gyro_x;
  fp32 chassis_gyro_y;
  fp32 chassis_gyro_z;

} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern void
chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set,
                             chassis_move_t *chassis_move_rc_to_vector);

#endif
