#include "chassis_odom_task.h"
#include "RobotProperties.h"
#include <FreeRTOS.h>
#include <stdint.h>
#include <arm_math.h>

extern motor_measure_t motor_chassis[7];

double currentTime = 0;
extern float AHRSYaw;

float wheelCircumference;

MecanumWheelValues lastVelocitites;
MecanumPosition currentPosition;

void initChassisOdom(){
    memset(&currentPosition, 0, sizeof(MecanumPosition));
    wheelCircumference = (WHEEL_RADIUS * 2.0f) * PI;
}

float convertRPMToMSChassis(float rpm){
    return ((rpm) / 36.0f / 60.0f) * wheelCircumference;
}

extern fp32 INS_angle[3];

int32_t total_ecd[4] = {0,0,0,0};
int32_t lastTotal_ecd[4] = {0,0,0,0};
int32_t delta_ecd[4] = {0,0,0,0};

float32_t ecdVx, ecdVy, ecdW;
float32_t distanceX, distanceY;

void chassisOdomUpdate(void const *argument){
    for(;;){
        //Update ECD values
        for(uint8_t i = 0; i < 4; i++){
            lastTotal_ecd[i] = total_ecd[i];
            total_ecd[i] = motor_chassis[i].total_ecd;
            delta_ecd[i] = total_ecd[i] - lastTotal_ecd[i];
        }

        ecdVx = -delta_ecd[0] + delta_ecd[1] + delta_ecd[2] - delta_ecd[3];
        ecdVy = -delta_ecd[0] - delta_ecd[1] + delta_ecd[2] + delta_ecd[3];
        ecdW = -delta_ecd[0] - delta_ecd[1] - delta_ecd[2] - delta_ecd[3];

        //TODO Get yaw
        fp32 currentYaw = INS_angle[0];
        float32_t yawSin = arm_sin_f32(currentYaw);
        float32_t yawCos = arm_cos_f32(currentYaw);

        distanceX += (yawCos * ecdVx - yawSin * ecdVy) * 0.25f * ENCODERCODES_TO_MS;
        distanceY += (yawSin * ecdVx + yawCos * ecdVy) * 0.25f * ENCODERCODES_TO_MS;	
				
        vTaskDelay(1);
    }

    vTaskDelete(NULL);
}   
