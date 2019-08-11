#include "autoMecanisim.h"
#include <remote_control.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "mecanisim_task.h"
#include "stm32f4xx.h"
#include "PWMUtils.h"

extern float cupThingieSetPoint;
extern float armSetPoint;
extern float servoSetPoint;

bool hasCup = false;

TickType_t desiredWaitTime;

bool waitingForMovement;
bool movementFinished;

AutoMecanisimGoal currentGoal;
CupGrabbingSteps cupGrabStatus;

extern TIM_HandleTypeDef htim2;

void initAutoMecanisim() {
  cupThingieSetPoint = CUP_THINGIE_RESTING_ANGLE;
  armSetPoint = ARM_RESTING_ANGLE;
  servoSetPoint = CLAW_READY_SET_POINT;

  desiredWaitTime = 0;

  currentGoal = GRABING_CUP;

  waitingForMovement = false;
  movementFinished = false;
}

void updateAutoMecanisim(void* arguments) {
  for (;;) {
    const RC_ctrl_t* rcCtrl = get_remote_control_point();
    int16_t scrollWheel = rcCtrl->rc.ch[4];

    if (!waitingForMovement || true) {
      switch (currentGoal) {
        default: { break; }

        case GRABING_CUP: {
          switch (cupGrabStatus) {
            case PREPARE_POSITION: {
              servoSetPoint = CLAW_READY_SET_POINT;
              armSetPoint = ARM_GET_CUP_ANGLE;
              cupThingieSetPoint = CUP_THINGIE_READY_ANGLE;
							

              //if (movementFinished) {
                // Move on
              //  movementFinished = false;
              //  cupGrabStatus = GRAB_CUP;
              //  break;
              //}
							

              //desiredWaitTime = 2000.0 + xTaskGetTickCount();
              //waitingForMovement = true;
							
							sendBallCANMessage();
							__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
							vTaskDelay(1200);
							
							cupGrabStatus = GRAB_CUP;


              break;
            }

            case GRAB_CUP: {
              servoSetPoint = CLAW_GRAB_SET_POINT;
              armSetPoint = ARM_GET_CUP_ANGLE;

              //if (movementFinished) {
              //  movementFinished = false;
              //  cupGrabStatus = LIFT_CUP;
              //  break;
              //}

							sendBallCANMessage();
							vTaskDelay(1000);
							__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
							vTaskDelay(1600);
							
							cupGrabStatus = LIFT_CUP;
							
              //desiredWaitTime = 2000.0 + xTaskGetTickCount();
              //waitingForMovement = true;
							

              break;
            }

            case LIFT_CUP: {
							servoSetPoint = CLAW_GRAB_SET_POINT;
              armSetPoint = ARM_CUP_UP_ANGLE;

              //.if (movementFinished) {
              //  movementFinished = false;
              //  cupGrabStatus = PREPARE_POSITION;
              //  currentGoal = REST;
              //  hasCup = true;
								
							//	break;
              //}

              //desiredWaitTime = 1000.0 + xTaskGetTickCount();
              //waitingForMovement = true;
							
							sendBallCANMessage();
							__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servoSetPoint);
							vTaskDelay(1000);
							
							currentGoal = REST;

              break;
            }
          }

          break;
        }
      }

      sendBallCANMessage();

    } else {
      if (xTaskGetTickCount() > desiredWaitTime) {
        waitingForMovement = false;
        movementFinished = true;
      }
    }

    vTaskDelay(40 / portTICK_PERIOD_MS);
  }
}
