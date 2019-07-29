#include "chassis_control.h"
#include "PID.h"
#include "chassis_task.h"

#include "arm_math.h"
#include "main.h"
#include "string.h"

#include "cmsis_os.h"

#include "MecanumKinematics.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00005f

extern QueueHandle_t recvMotorQueue;

uint8_t chassis_odom_pack_solve(uint8_t *buf, float x, float y, float odom_yaw,
                                float vx, float vy, float vw, float gyro_z,
                                float gyro_yaw) {
  return 0;
}
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

//底盘运动数据
chassis_move_t chassis_move;
uint8_t usb_tx[128];

void chassis_motor_speed_update(chassis_move_t *chassis_move_update) {
  uint8_t i = 0;
  for (i = 0; i < 4; i++) {
    //更新电机速度，加速度是速度的PID微分
    chassis_move_update->motor_chassis[i].speed =
        CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *
        chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
  }
}

void chassis_twist_to_mecanum_wheel_speed(Twist2D *input, fp32 wheel_speed[4]) {
  MecanumWheelValues values;
  mecanumInverseKinematics(input, 2, 1, &values);

  wheel_speed[0] = values.topLeft;
  wheel_speed[1] = values.topRight;
  wheel_speed[2] = values.backLeft;
  wheel_speed[3] = values.backRight;
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

Twist2D recev;

void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set,
                          chassis_move_t *chassis_move_rc_to_vector) {
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL ||
      chassis_move_rc_to_vector == NULL) {
    return;
  }

  // Twist2D recev;
  xQueueReceive(recvMotorQueue, &(recev), (TickType_t)10);

  (*vx_set) = recev.vX;
  (*vy_set) = recev.vY;
  (*wz_set) = recev.w;

  return;
}

void chassis_PID_init(void) {
  //底盘速度环pid值
  const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP,
                                          M3505_MOTOR_SPEED_PID_KI,
                                          M3505_MOTOR_SPEED_PID_KD};

  const static fp32 chassis_rotation_pid[3] = {CHASSIS_ROTATION_PID_KP,
                                               CHASSIS_ROTATION_PID_KI,
                                               CHASSIS_ROTATION_PID_KD};
  //底盘旋转环pid值
  const static fp32 chassis_angle_pid[3] = {
      CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};

  uint8_t i;

  //初始化PID 运动
  for (i = 0; i < 4; i++) {
    PID_Init(&chassis_move.motor_speed_pid[i], PID_POSITION, motor_speed_pid,
             M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
  }

  //初始化旋转PID
  PID_Init(&chassis_move.chassis_rotation_pid, PID_POSITION,
           chassis_rotation_pid, CHASSIS_ROTATION_PID_MAX_OUT,
           CHASSIS_ROTATION_PID_MAX_IOUT);

  PID_Init(&chassis_move.chassis_angle_pid, PID_POSITION, chassis_angle_pid,
           CHASSIS_ANGLE_PID_MAX_OUT, CHASSIS_ANGLE_PID_MAX_IOUT);
}
