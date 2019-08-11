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
#include <arm_math.h>
#include "MecanumKinematics.h"

extern CAN_HandleTypeDef hcan2;
extern QueueHandle_t canTestTransmitQueue;

extern float32_t distanceX, distanceY;
extern float vX, vY, vW;
extern fp32 INS_angle[3];

void canTransmitTaskLoop(void const *argument){
    for (;;){
        if(HAL_CAN_GetTxMailboxesFreeLevel(&MANIFOLD_CAN) > 0){
          //Gyro
          //canSendFloatMessage(CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_ROLL, AHRSRoll);
          //canSendFloatMessage(CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_PITCH, AHRSPitch);
          //canSendFloatMessage(CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_YAW, AHRSYaw);

          canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_X, distanceX, 0);
          canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_Y, distanceY, 0);
          canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_YAW, INS_angle[0], 0);	
					//vTaskDelay(10 / portTICK_PERIOD_MS); 
          
          //canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_VXY, vX, vY);
          //canSendFloatMessage(CANMESSAGE_ID_ODOMETRY, CANMESSAGE_SUBID_ODOM_VW, vW, 0);	
        }
				
        vTaskDelay(15 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

uint32_t canTxMailbox;
CAN_TxHeaderTypeDef msgHeader;
uint8_t send_data[8];

void canSendFloatMessage(uint8_t id, uint8_t subid, float data, float data2){
    msgHeader.StdId = calculateId(id, subid);
    msgHeader.IDE = CAN_ID_STD;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.DLC = 0x08;
		
		serializeFloat(data, send_data);
    serializeFloat(data2, send_data + 4);

    HAL_CAN_AddTxMessage(&MANIFOLD_CAN, &msgHeader, (uint8_t*) &send_data,
                       &canTxMailbox);
}

