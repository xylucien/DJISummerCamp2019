#include "autoMecanisim.h"
#include <remote_control.h>
#include <stdbool.h>
#include "PWMUtils.h"
#include "cmsis_os.h"
#include "mecanisim_task.h"
#include "stm32f4xx.h"

extern float cupThingieSetPoint;
extern float armSetPoint;
extern float servoSetPoint;

int16_t currentCupCount;

extern float centerSetPoint;
int16_t currentBallCout;

extern TIM_HandleTypeDef htim2;

bool hasCup = false;

bool waitingForMovement;
bool movementFinished;

AutoMecanisimGoal currentGoal;
CupGrabbingSteps cupGrabStatus;

void initAutoMecanisim() {
  cupThingieSetPoint = CUP_THINGIE_RESTING_ANGLE;
  armSetPoint = ARM_RESTING_ANGLE;
  servoSetPoint = CLAW_READY_SET_POINT;

  currentGoal = REST;
	cupGrabStatus = DROPPING_BALL;

  waitingForMovement = false;
  movementFinished = false;

  currentBallCout = 0;
  currentCupCount = 9;
}

void updateAutoMecanisim(void* arguments) {
  for (;;) {
    const RC_ctrl_t* rcCtrl = get_remote_control_point();
    int16_t scrollWheel = rcCtrl->rc.ch[4];

    switch (currentGoal) {

      case PLACE_CUP: {
        servoSetPoint = CLAW_RELEASE_SET_POINT;
        armSetPoint = 0;

        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
        sendBallCANMessage();

        currentGoal = REST;

        break;
      }

      default: { break; }

      case GRABING_CUP: {
        switch (cupGrabStatus) {
          case DROPPING_BALL: {
            servoSetPoint = CLAW_READY_SET_POINT;
            armSetPoint = ARM_RESTING_ANGLE;
            cupThingieSetPoint = CUP_THINGIE_GET_BALL;

            currentBallCout++;
            centerSetPoint += 90.0f;

            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
						__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, cupThingieSetPoint);
						vTaskDelay(1000);
            sendBallCANMessage();
            vTaskDelay(1500);

            cupGrabStatus = PREPARE_POSITION;

            break;
          }

          case PREPARE_POSITION: {
            servoSetPoint = CLAW_READY_SET_POINT;
            armSetPoint = ARM_GET_CUP_ANGLE;

            if(currentCupCount > 5){
              cupThingieSetPoint = CUP_THINGIE_READY_ANGLE_10;
            } else {
              cupThingieSetPoint = CUP_THINGIE_READY_ANGLE_5;
            }

            currentCupCount--;

            if(currentCupCount < 0){
              //Loop back
              currentCupCount = 10;
            }

            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, cupThingieSetPoint);
						vTaskDelay(500);
            sendBallCANMessage();
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
            vTaskDelay(1200);

            cupGrabStatus = GRAB_CUP;

            break;
          }

          case GRAB_CUP: {
            servoSetPoint = CLAW_GRAB_SET_POINT;
            armSetPoint = ARM_GET_CUP_ANGLE;

            sendBallCANMessage();
            vTaskDelay(500);
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
            vTaskDelay(1600);

            cupGrabStatus = LIFT_CUP;

            break;
          }

          case LIFT_CUP: {
            servoSetPoint = CLAW_GRAB_SET_POINT;
            armSetPoint = ARM_CUP_UP_ANGLE;
            cupGrabStatus = DROPPING_BALL;
						//cupThingieSetPoint = CUP_THINGIE_RESTING_ANGLE;

            //.if (movementFinished) {
            //  movementFinished = false;
            //  cupGrabStatus = PREPARE_POSITION;
            //  currentGoal = REST;
            //  hasCup = true;

            //	break;
            //}

            // desiredWaitTime = 1000.0 + xTaskGetTickCount();
            // waitingForMovement = true;

            sendBallCANMessage();
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, cupThingieSetPoint);
            vTaskDelay(1000);

            currentGoal = REST;

            break;
          }
        }

        break;
      }
    }

    sendBallCANMessage();

    vTaskDelay(40 / portTICK_PERIOD_MS);
  }
}
