#include "mecanisim_task.h"
#include "stm32f4xx.h"
#include "PositionPID.h"
#include <stdlib.h>
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "arm_math.h"

extern CAN_HandleTypeDef hcan1;
PositionPIDData testData;

void initMecanisimTask() {
    testData.velocityPid = malloc(sizeof(arm_pid_instance_f32));

    testData.velocityPid->Kp = 120.1;
    testData.velocityPid->Ki = 0.0;
    testData.velocityPid->Kd = 0.0;
		testData.maximumVelocity = 10000;

    testData.positionPid = malloc(sizeof(arm_pid_instance_f32));

    testData.positionPid->Kp = 0.01;
    testData.positionPid->Ki = 0.0;
    testData.positionPid->Kd = 0.0;
		testData.positionLimitEnabled = false;

    initializePositionPid(&testData);   
}

extern motor_measure_t motor_mecanisim[8];
extern int16_t motor8Set;
float testSetPoint;

void mecanisimTaskUpdate(void *arguments){
    for(;;){
        motor8Set = calculatePositionPid(&testData, motor_mecanisim[0].speed_rpm, motor_mecanisim[0].total_ecd, testSetPoint);
        vTaskDelay(1);
    }
}
