/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief ң����������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  * ��Դ�����ô��ڿ����ж���������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
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

#include "Remote_Control.h"
#include "bsp_remote_control.h"
#include "main.h"

//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700

//ȡ������
static int16_t RC_abs(int16_t value);
//ң������������
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//ң�������Ʊ���
static RC_ctrl_t rc_ctrl;
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

//��ʼ��DMA������1
void remote_control_init(void) {
  RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}
//����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const RC_ctrl_t *get_remote_control_point(void) { return &rc_ctrl; }

//�ж�ң���������Ƿ������
uint8_t RC_data_is_error(void) {
  //ʹ����go to��� �������ͳһ����ң�����������ݹ���
  if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
    goto error;
  }
  if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
    goto error;
  }
  if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
    goto error;
  }
  if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
    goto error;
  }
  if (rc_ctrl.rc.s[0] == 0) {
    goto error;
  }
  if (rc_ctrl.rc.s[1] == 0) {
    goto error;
  }
  return 0;

error:
  rc_ctrl.rc.ch[0] = 0;
  rc_ctrl.rc.ch[1] = 0;
  rc_ctrl.rc.ch[2] = 0;
  rc_ctrl.rc.ch[3] = 0;
  rc_ctrl.rc.ch[4] = 0;
  rc_ctrl.rc.s[0] = RC_SW_DOWN;
  rc_ctrl.rc.s[1] = RC_SW_DOWN;
  rc_ctrl.mouse.x = 0;
  rc_ctrl.mouse.y = 0;
  rc_ctrl.mouse.z = 0;
  rc_ctrl.mouse.press_l = 0;
  rc_ctrl.mouse.press_r = 0;
  rc_ctrl.key.v = 0;
  return 1;
}

void solve_RC_lost(void) { RC_restart(SBUS_RX_BUF_NUM); }
void solve_data_error(void) { RC_restart(SBUS_RX_BUF_NUM); }

//�����ж�
void USART1_IRQHandler(void) {
  static volatile uint8_t res;
  if (USART1->SR & (1 << 5)) //���յ�����
  {
    res = USART1->DR;

  } else if (USART1->SR & (1 << 4)) {
    static uint16_t this_time_rx_len = 0;
    res = USART1->DR;
    if ((DMA2_Stream2->CR & DMA_SxCR_CT) != 0) {
      DMA2_Stream2->CR &= ~(1 << 0);
      this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream2->NDTR;
      DMA2_Stream2->NDTR = SBUS_RX_BUF_NUM;
      DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
      DMA2_Stream2->CR |= 1 << 0;

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        //����ң��������
        SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
      }
    } else {
      DMA2_Stream2->CR &= ~(1 << 0);
      this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream2->NDTR;
      DMA2_Stream2->NDTR = SBUS_RX_BUF_NUM;
      DMA2_Stream2->CR |= DMA_SxCR_CT;
      DMA2_Stream2->CR |= 1 << 0;

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        //����ң��������
        SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
      }
    }
  }
}

//ȡ������
static int16_t RC_abs(int16_t value) {
  if (value > 0) {
    return value;
  } else {
    return -value;
  }
}
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
  if (sbus_buf == NULL || rc_ctrl == NULL) {
    return;
  }

  rc_ctrl->rc.ch[0] =
      (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
  rc_ctrl->rc.ch[1] =
      ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;        //!< Channel 1
  rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | //!< Channel 2
                       (sbus_buf[4] << 10)) &
                      0x07ff;
  rc_ctrl->rc.ch[3] =
      ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
  rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);       //!< Switch left
  rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;  //!< Switch right
  rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);    //!< Mouse X axis
  rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);    //!< Mouse Y axis
  rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);  //!< Mouse Z axis
  rc_ctrl->mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press ?
  rc_ctrl->mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press ?
  rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);    //!< KeyBoard value
  rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); // NULL

  rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
