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

void canTransmitTaskLoop(void const *argument){
    for (;;){
        //if(HAL_CAN_GetTxMailboxesFreeLevel(&MANIFOLD_CAN) > 0){
				
        //}
				
        vTaskDelay(66);
    }

    vTaskDelete(NULL);
}

uint32_t canTxMailbox;
CAN_TxHeaderTypeDef msgHeader;
uint8_t send_data[8];

void canSendFloatMessage(uint8_t id, uint8_t subid, float data){
    msgHeader.StdId = calculateId(id, subid);
    msgHeader.IDE = CAN_ID_STD;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.DLC = 0x08;
		
		serializeFloat(data, send_data);

    HAL_CAN_AddTxMessage(&MANIFOLD_CAN, &msgHeader, (uint8_t*) &send_data,
                       &canTxMailbox);
}

