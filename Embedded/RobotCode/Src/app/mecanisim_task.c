#include "mecanisim_task.h"
#include <remote_control.h>
#include <stdlib.h>
#include "CAN_receive.h"
#include "PositionPID.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"

#define C610ANGLETOCODES 819.9f

extern CAN_HandleTypeDef hcan2;

PositionPIDData rightBallThingie;
PositionPIDData centerBallThingie;
PositionPIDData leftBallThingie;

void initMecanisimTask() { ; }

// Teleop Balls
bool ballCANMode = false;

bool previouslyUpSwitch = false;
bool previouslyDownSwitch = false;

float32_t currentBallCountRight = 0;
float32_t currentBallCountLeft = 0;

float rightSetPoint = 0.0f;
float centerSetPoint = 0.0f;
float leftSetPoint = 0.0f;

void mecanisimTaskUpdate(void* arguments) {
  for (;;) {
    // Ballz
    if (!ballCANMode) {
      const RC_ctrl_t* rcCtrl = get_remote_control_point();

      bool upSwitch = switch_is_up(rcCtrl->rc.s[1]);
      bool downSwitch = switch_is_down(rcCtrl->rc.s[1]);

      if (previouslyUpSwitch != upSwitch && upSwitch) {
        currentBallCountRight += 1.0f;
      } else if (previouslyDownSwitch != downSwitch && downSwitch) {
        // Increment left
        currentBallCountLeft += 1.0f;
      }

      previouslyUpSwitch = upSwitch;
      previouslyDownSwitch = downSwitch;

      rightSetPoint = currentBallCountRight * 90.0f;
      leftSetPoint = -currentBallCountLeft * 90.0f;
    }

		//rightNativeSetPoint = 1000;
		//centerNativeSetPoint = 2000;
		//leftNativeSetPoint = 3000;
		
    sendBallCANMessage();

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// 0-1 Right set point (codes)
// 2-3 Center set point (codes)
// 4-5 Left set point
// 6-7 000000000000000
void sendBallCANMessage() {
  uint32_t canTxMailbox;
  CAN_TxHeaderTypeDef msgHeader;
  uint8_t send_data[8];

  msgHeader.StdId = 0x700;
  msgHeader.IDE = CAN_ID_STD;
  msgHeader.RTR = CAN_RTR_DATA;
  msgHeader.DLC = 0x08;

  memcpy(send_data, &rightSetPoint, sizeof(float));
  memcpy(send_data + sizeof(float), &leftSetPoint, sizeof(float));
  //memcpy(send_data + 4, &leftSetPoint, sizeof(float));
  //memset(send_data + 6, 0, sizeof(float));

  HAL_CAN_AddTxMessage(&hcan2, &msgHeader, (uint8_t*) &send_data,
                       &canTxMailbox);

  msgHeader.StdId = 0x701;
  msgHeader.IDE = CAN_ID_STD;
  msgHeader.RTR = CAN_RTR_DATA;
  msgHeader.DLC = 0x08;

  memcpy(send_data, &centerSetPoint, sizeof(float));
  //memcpy(send_data + sizeof(float), &centerSetPoint, sizeof(float));
  //memcpy(send_data + 4, &leftSetPoint, sizeof(float));
  memset(send_data + sizeof(float), 0, sizeof(float));

  HAL_CAN_AddTxMessage(&hcan2, &msgHeader, (uint8_t*) &send_data,
                       &canTxMailbox);
}

