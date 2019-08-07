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

    testData.velocityPid->Kp = 1000.1;
    testData.velocityPid->Ki = 0.0;
    testData.velocityPid->Kd = 0.0;
		testData.maximumVelocity = 10000;

    testData.positionPid = malloc(sizeof(arm_pid_instance_f32));

    testData.positionPid->Kp = 0.7;
    testData.positionPid->Ki = 0.0;
    testData.positionPid->Kd = 0.0;
		testData.positionLimitEnabled = false;

    initializePositionPid(&testData);   
}

extern motor_measure_t motor_chassis[8];

int16_t motor4Set = 1000.0;
int16_t motor5Set = 0.0;
int16_t motor6Set = 0.0;
int16_t motor7Set = 0.0;

static uint8_t txMessage[8];

void mecanisimTaskUpdate(void *arguments){
    for(;;){
        for(float32_t i = -1.0f; i < 1.0f; i = i + .0001){
						fp32 setPoint = arm_sin_f32(i) * 1000.0f;
            float32_t motorSet = calculatePositionPid(&testData, motor_chassis[4].speed_rpm, motor_chassis[4].total_ecd, setPoint);
            motor4Set = motorSet;

            updateMotors();
            vTaskDelay(1);
        }
    }
}


void updateMotors(){
    static CAN_TxHeaderTypeDef txCANHeader;
    uint32_t send_mailBox;

    txCANHeader.StdId = 0x1FF;
    txCANHeader.IDE = CAN_ID_STD;
    txCANHeader.RTR = CAN_RTR_DATA;
    txCANHeader.DLC = 0x08;

    txMessage[0] = motor4Set >> 8;
    txMessage[1] = motor4Set;
    txMessage[2] = motor5Set >> 8;
    txMessage[3] = motor5Set;
    txMessage[4] = motor6Set >> 8;
    txMessage[5] = motor6Set;
    txMessage[6] = motor7Set >> 8;
    txMessage[7] = motor7Set;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &txCANHeader, txMessage,
                       &send_mailBox);
}

