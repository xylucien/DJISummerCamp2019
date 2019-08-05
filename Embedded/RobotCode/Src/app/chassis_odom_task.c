#include "chassis_odom_task.h"
#include "RobotProperties.h"
#include <FreeRTOS.h>

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

void chassisOdomUpdate(void const *argument){
    for(;;){
        currentTime = (HAL_GetTick() * portTICK_PERIOD_MS) / 1000.0f;

        lastVelocitites.topRight = convertRPMToMSChassis(motor_chassis[0].speed_rpm);
        lastVelocitites.topLeft = convertRPMToMSChassis(motor_chassis[1].speed_rpm);

        lastVelocitites.backLeft = convertRPMToMSChassis(motor_chassis[2].speed_rpm);
        lastVelocitites.backRight = convertRPMToMSChassis(motor_chassis[3].speed_rpm);

        //TODO Get gyro yaw
        float currentYaw = 0;
        integrateVelocities(&lastVelocitites, AB, currentTime, currentYaw, &currentPosition);
    }

    vTaskDelete(NULL);
}   
