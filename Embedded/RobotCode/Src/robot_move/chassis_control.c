#include "chassis_control.h"
#include "PID.h"
#include "chassis_task.h"

#include "arm_math.h"
#include "main.h"
#include "string.h"

#include "cmsis_os.h"
#include <stdbool.h>

#include "MecanumKinematics.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00005f

extern QueueHandle_t recvMotorQueue;

extern QueueHandle_t canTargetVelocityQueue;

extern float lastVx;
extern float lastVy;
extern float lastVw;

Twist2D currentVelocityTarget;

uint8_t chassis_odom_pack_solve(uint8_t *buf, float x, float y, float odom_yaw,
                                float vx, float vy, float vw, float gyro_z,
                                float gyro_yaw) {
  return 0;
}
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

//�����˶�����
chassis_move_t chassis_move;
uint8_t usb_tx[128];

void chassis_motor_speed_update(chassis_move_t *chassis_move_update) {
  uint8_t i = 0;
  for (i = 0; i < 4; i++) {
    //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
    chassis_move_update->motor_chassis[i].speed =
        CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *
        chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
  }
}

float topLeft;
float topRight;
float backLeft;
float backRight;

void chassis_twist_to_mecanum_wheel_speed(Twist2D *input, fp32 *wheelSpeed) {
  MecanumWheelValues values;
  mecanumInverseKinematics(input, 2, 1, &values);

  topLeft = values.topLeft;
  topRight = values.topRight;
  backLeft = values.backLeft;
  backRight = values.backRight;
	
	wheelSpeed[0] = -values.topRight;
	wheelSpeed[1] = values.topLeft;
	wheelSpeed[2] = values.backLeft;
	wheelSpeed[3] = -values.backRight;
}

void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set,
                                           const fp32 wz_set,
                                           fp32 wheel_speed[4]) {
  Twist2D input;

  input.vX = vx_set;
  input.vY = vy_set;
  input.w = wz_set;

  chassis_twist_to_mecanum_wheel_speed(&input, wheel_speed);
}

static fp32 distance_x = 0.0f, distance_y = 0.0f, distance_wz = 0.0f;
void chassis_distance_calc_task(void const *argument) {

  while (1) {

    osDelay(1);
  }
}

void chassis_distance_send_task(void const *argument) {

  while (1) {
    uint8_t send_len;
    send_len = chassis_odom_pack_solve(
        usb_tx, distance_x, distance_y, distance_wz, chassis_move.vx,
        chassis_move.vy, chassis_move.wz, chassis_move.chassis_gyro_z,
        chassis_move.chassis_yaw);
    CDC_Transmit_FS(usb_tx, send_len);
    osDelay(10);
  }
}

void chassis_normal_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set,
                            chassis_move_t *chassis_move_rc_to_vector) {
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL ||
      chassis_move_rc_to_vector == NULL) {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
  *wz_set = -CHASSIS_WZ_RC_SEN *
            chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

float a = 0.13;
float b = 0.14;
float radius = 0.04;

void chassis_twist_control(Twist2D *twist, chassis_move_t *chassis_move){
  MecanumWheelValues values;
  mecanumInverseKinematics(twist, a + b, radius, &values);
	
	topLeft = values.topLeft;
  topRight = values.topRight;
  backLeft = values.backLeft;
  backRight = values.backRight;

  chassis_move->motor_chassis[0].speed_set = values.topLeft;
  chassis_move->motor_chassis[1].speed_set = values.topRight;
  chassis_move->motor_chassis[2].speed_set = values.backLeft;
  chassis_move->motor_chassis[3].speed_set = values.backRight;
}

Twist2D targetVel;
fp32 *wheel_speedsTest[4];

void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set,
                          chassis_move_t *chassis_move_rc_to_vector) {
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL ||
      chassis_move_rc_to_vector == NULL) {
    //return;
				;
			}

  targetVel.vX = lastVx;
  targetVel.vY = lastVy;
  targetVel.w = lastVw;

	(*vx_set) = targetVel.vX;
	(*vy_set) = targetVel.vY;
	(*wz_set) = targetVel.w;

  chassis_twist_control(&targetVel, chassis_move_rc_to_vector);

  return;
}

void chassis_PID_init(void) {
  //�����ٶȻ�pidֵ
  const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP,
                                          M3505_MOTOR_SPEED_PID_KI,
                                          M3505_MOTOR_SPEED_PID_KD};

  const static fp32 chassis_rotation_pid[3] = {CHASSIS_ROTATION_PID_KP,
                                               CHASSIS_ROTATION_PID_KI,
                                               CHASSIS_ROTATION_PID_KD};
  //������ת��pidֵ
  const static fp32 chassis_angle_pid[3] = {
      CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};

  uint8_t i;

  //��ʼ��PID �˶�
  for (i = 0; i < 4; i++) {
    PID_Init(&chassis_move.motor_speed_pid[i], PID_POSITION, motor_speed_pid,
             M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
  }

  //��ʼ����תPID
  PID_Init(&chassis_move.chassis_rotation_pid, PID_POSITION,
           chassis_rotation_pid, CHASSIS_ROTATION_PID_MAX_OUT,
           CHASSIS_ROTATION_PID_MAX_IOUT);

  PID_Init(&chassis_move.chassis_angle_pid, PID_POSITION, chassis_angle_pid,
           CHASSIS_ANGLE_PID_MAX_OUT, CHASSIS_ANGLE_PID_MAX_IOUT);
}
