#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "fifo.h"
#  include "protocol.h"
#include "referee.h"
#include "stdio.h"
#include "string.h"
#include "RobotProperties.h"

#include "MecanumKinematics.h"

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
extern void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

// USB�ײ㷢�ͺ�����ֱ�Ӳ���Ӳ��
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

//����ʽ��������
communicate_class_input_data_t communicate_input_data;
//����ʽ������
communicate_class_output_data_t communicate_output_data;

chassis_ctrl_info_t chassis_ctrl_input_data;

extern QueueHandle_t recvMotorQueue;

//ʵ��RMЭ������л�����
void referee_send_data(uint16_t cmd_id, void *buf, uint16_t len) {
  // TODO ��������128�ֽڴ�С��������
  static uint8_t send_buf[128];
  uint16_t index = 0;
  // TODO ����֡ͷ�ṹ��
  frame_header_struct_t referee_send_header;

  // TODO ��ʼ����Ӧ֡ͷ�ṹ��
  referee_send_header.SOF = HEADER_SOF;
  referee_send_header.data_length = len;
  referee_send_header.seq++;

  // TODO ����CRC8У��
  Append_CRC8_Check_Sum((uint8_t *)&referee_send_header,
                        sizeof(frame_header_struct_t));

  memcpy(send_buf, (uint8_t *)&referee_send_header,
         sizeof(frame_header_struct_t));
  index += sizeof(frame_header_struct_t);

  memcpy(send_buf + index, (void *)&cmd_id, sizeof(uint16_t));
  index += sizeof(uint16_t);

  // TODO ������ݰ�
  memcpy(send_buf + index, (void *)buf, len);
  index += len;

  // TODO ����CRC16У��
  Append_CRC16_Check_Sum(send_buf, REF_HEADER_CRC_CMDID_LEN + len);
  index += sizeof(uint16_t);

  // TODO ���õײ㷢�ͺ���
  CDC_Transmit_FS(send_buf, index);
}

// TODO��ʵ�ּ���ʽ����ͽ���Ϸ�
//����ʽ��Ϣ�������л��Ѿ�ʵ��
//ͨ��communicate_input_data�ṹ�����ֱ�ӻ�ȡ����ʽ��Ϣ
// operate '+' '-' '*' '/'
void communicate_class_solve(void) { ; }

void chassis_command_solve() {
  Twist2D input;

  input.vX = chassis_ctrl_input_data.vx;
  input.vY = chassis_ctrl_input_data.vy;
  input.w = chassis_ctrl_input_data.vw;

  MecanumWheelValues values;

  mecanumInverseKinematics(&input, AB, WHEEL_RADIUS, &values);

  xQueueSend(recvMotorQueue, (void *)(&input), (TickType_t)10);
}
