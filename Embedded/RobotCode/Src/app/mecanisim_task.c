#include "mecanisim_task.h"
#include "stm32f4xx.h"
#include "PositionPID.h"
#include <stdlib.h>
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include <remote_control.h>

extern CAN_HandleTypeDef hcan1;
PositionPIDData rightBallThingie;
PositionPIDData leftBallThingie;

void initMecanisimTask() {
  //Right ball thingie
  rightBallThingie.velocityPid = malloc(sizeof(arm_pid_instance_f32));

  rightBallThingie.velocityPid->Kp = 30.1;
  rightBallThingie.velocityPid->Ki = 0.0;
  rightBallThingie.velocityPid->Kd = 0.0;
  rightBallThingie.maximumVelocity = 10000;

  rightBallThingie.positionPid = malloc(sizeof(arm_pid_instance_f32));

  rightBallThingie.positionPid->Kp = 0.00085;
  rightBallThingie.positionPid->Ki = 0.0;
  rightBallThingie.positionPid->Kd = 0.0;
  rightBallThingie.positionLimitEnabled = false;

  //Left ball thingie
  leftBallThingie.velocityPid = malloc(sizeof(arm_pid_instance_f32));

  leftBallThingie.velocityPid->Kp = 30.1;
  leftBallThingie.velocityPid->Ki = 0.0;
  leftBallThingie.velocityPid->Kd = 0.0;
  leftBallThingie.maximumVelocity = 10000;

  leftBallThingie.positionPid = malloc(sizeof(arm_pid_instance_f32));

  leftBallThingie.positionPid->Kp = 0.0007;
  leftBallThingie.positionPid->Ki = 0.0;
  leftBallThingie.positionPid->Kd = 0.0;
  leftBallThingie.positionLimitEnabled = false;

  initializePositionPid(&rightBallThingie);   
  initializePositionPid(&leftBallThingie);
}

extern bool new8_11data;
extern motor_measure_t motor_mecanisim[8];

extern int16_t motor8Set;
extern int16_t motor9Set;

float rightSetPoint;
float leftSetPoint;

#define C610ANGLETOCODES 819.9f

bool ballCANMode = false;

bool previouslyUpSwitch = false;
bool previouslyDownSwitch = false;

float32_t currentBallCountRight = 0;
float32_t currentBallCountLeft = 0;

void mecanisimTaskUpdate(void *arguments){
    for(;;){
				float32_t rightNativeSetPoint = 0;
				float32_t leftNativeSetPoint = 0;
			
				if(ballCANMode){
					rightNativeSetPoint = rightSetPoint * C610ANGLETOCODES;
					leftNativeSetPoint = leftSetPoint * C610ANGLETOCODES;
				} else {
          const RC_ctrl_t* rcCtrl = get_remote_control_point();

					bool upSwitch = switch_is_up(rcCtrl->rc.s[1]);
					bool downSwitch = switch_is_down(rcCtrl->rc.s[1]);
					
					if(previouslyUpSwitch != upSwitch && upSwitch){
						currentBallCountRight += 1.0f;
					} else if (previouslyDownSwitch != downSwitch && downSwitch) {
						//Increment left
						currentBallCountLeft += 1.0f;
					}
					
					previouslyUpSwitch = upSwitch;
					previouslyDownSwitch = downSwitch;
					
					rightNativeSetPoint = currentBallCountRight * 90.0f * C610ANGLETOCODES;
					leftNativeSetPoint = -currentBallCountLeft * 90.0f * C610ANGLETOCODES;
				}
			
        motor8Set = calculatePositionPid(&rightBallThingie, (float32_t) motor_mecanisim[0].speed_rpm / 36.0, (float32_t) motor_mecanisim[0].total_ecd, rightNativeSetPoint);
        motor9Set = calculatePositionPid(&leftBallThingie, (float32_t) motor_mecanisim[1].speed_rpm / 36.0, (float32_t) motor_mecanisim[1].total_ecd, leftNativeSetPoint);

        new8_11data = true;
        vTaskDelay(20);
    }
}
