#include "mecanisim_task.h"
#include "stm32f4xx.h"
#include "PositionPID.h"
#include <stdlib.h>
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include <remote_control.h>

#define C610ANGLETOCODES 819.9f

PositionPIDData rightBallThingie;
PositionPIDData centerBallThingie;
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

  //Center ball thingie
  centerBallThingie.velocityPid = malloc(sizeof(arm_pid_instance_f32));

  centerBallThingie.velocityPid->Kp = 30.1;
  centerBallThingie.velocityPid->Ki = 0.0;
  centerBallThingie.velocityPid->Kd = 0.0;
  centerBallThingie.maximumVelocity = 10000;

  centerBallThingie.positionPid = malloc(sizeof(arm_pid_instance_f32));

  centerBallThingie.positionPid->Kp = 0.00085;
  centerBallThingie.positionPid->Ki = 0.0;
  centerBallThingie.positionPid->Kd = 0.0;
  centerBallThingie.positionLimitEnabled = false;

  //Left ball thingie
  leftBallThingie.velocityPid = malloc(sizeof(arm_pid_instance_f32));

  leftBallThingie.velocityPid->Kp = 30.1;
  leftBallThingie.velocityPid->Ki = 0.0;
  leftBallThingie.velocityPid->Kd = 0.0;
  leftBallThingie.maximumVelocity = 10000;

  leftBallThingie.positionPid = malloc(sizeof(arm_pid_instance_f32));

  leftBallThingie.positionPid->Kp = 0.00085;
  leftBallThingie.positionPid->Ki = 0.0;
  leftBallThingie.positionPid->Kd = 0.0;
  leftBallThingie.positionLimitEnabled = false;

  initializePositionPid(&rightBallThingie);   
  initializePositionPid(&centerBallThingie);
  initializePositionPid(&leftBallThingie);
}

//Setpoints
float rightSetPoint = 0;
float centerSetPoint = 0;
float leftSetPoint = 0;

extern bool new8_11data;
extern bool new12_15data;
extern motor_measure_t motor_mecanisim[8];
extern int16_t motor8Set;
extern int16_t motor9Set;
extern int16_t motor12Set;

//Teleop Balls
bool ballCANMode = false;

bool previouslyUpSwitch = false;
bool previouslyDownSwitch = false;

float32_t currentBallCountRight = 0;
float32_t currentBallCountLeft = 0;

void mecanisimTaskUpdate(void *arguments){
    for(;;){
				float32_t rightNativeSetPoint = 0;
        float32_t centerNativeSetPoint = 0;
				float32_t leftNativeSetPoint = 0;
			
        //Ballz
				if(ballCANMode){
					rightNativeSetPoint = rightSetPoint * C610ANGLETOCODES;
          centerNativeSetPoint = centerSetPoint * C610ANGLETOCODES;
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
        motor12Set = calculatePositionPid(&centerBallThingie, (float32_t) motor_mecanisim[2].speed_rpm / 36.0, (float32_t) motor_mecanisim[2].total_ecd, centerNativeSetPoint);
        motor9Set = calculatePositionPid(&leftBallThingie, (float32_t) motor_mecanisim[1].speed_rpm / 36.0, (float32_t) motor_mecanisim[1].total_ecd, leftNativeSetPoint);
        new8_11data = true;
				new12_15data = true;

        vTaskDelay(20);
    }
}
