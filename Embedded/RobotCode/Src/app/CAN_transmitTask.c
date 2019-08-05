#include "CAN_transmitTask.h"
#include "CAN_receive.h"
#include "CANMessage.h"
#include "bsp_rng.h"
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "string.h"
#include "task.h"
#include "math.h"

#include "cmsis_os.h"

#include "CANMessage.h"
#include "CANUtil.h"
#include <stdio.h>
#include "AHRS_task.h"
#include "MecanumKinematics.h"

extern CAN_HandleTypeDef hcan2;
extern QueueHandle_t canTestTransmitQueue;

extern MecanumPosition currentPosition;

extern float AHRSRoll;
extern float AHRSPitch;
extern float AHRSYaw;

void canTransmitTaskLoop(void const *argument){
    for (;;){
				//Gyro
        canSendFloatMessage(CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_ROLL, AHRSRoll);
        canSendFloatMessage(CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_PITCH, AHRSPitch);
        canSendFloatMessage(CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_YAW, AHRSYaw);

        canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_X, currentPosition.x);
        canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_Y, currentPosition.y);
        canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_YAW, currentPosition.yaw);
    }

    vTaskDelete(NULL);
}

uint32_t canTxMailbox;
CAN_TxHeaderTypeDef msgHeader;

void canSendFloatMessage(uint8_t id, uint8_t subid, float data){
    msgHeader.StdId = calculateId(id, subid);
    msgHeader.IDE = CAN_ID_STD;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.DLC = 0x08;
		
		uint8_t send_data[8];
		serializeFloat(data, send_data);

    HAL_CAN_AddTxMessage(&MANIFOLD_CAN, &msgHeader, (uint8_t*) &send_data,
                       &canTxMailbox);
}

