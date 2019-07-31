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

extern CAN_HandleTypeDef hcan2;
extern QueueHandle_t canTestTransmitQueue;

void canTransmitTaskLoop(void const *argument){
    for (;;){
					for(float i = -1.0f; i < 1.0f; i += 0.1f){
						float output = i;
						xQueueSend(canTestTransmitQueue, (void *)(&output), (TickType_t)10);
						canSendTestMessage();
						vTaskDelay(1000);
					}
					
    }

    //vTaskDelete(NULL);
}

uint32_t canTxMailbox;
CAN_TxHeaderTypeDef testMsgHeader;

void canSendTestMessage(){
		float data;
    xQueueReceive(canTestTransmitQueue, (void*) &(data), (TickType_t)0);

    CANMessage message;
    message.messageId = CANMESSAGE_ID_TEST;

    memcpy(&message.data, &data, CANMESSAGE_ID_TEST_MSG_SIZE);

    testMsgHeader.StdId = CAN_MANIFOLD_ID;
    testMsgHeader.IDE = CAN_ID_STD;
    testMsgHeader.RTR = CAN_RTR_DATA;
    testMsgHeader.DLC = 0x08;
		
		uint8_t send_data[8];
		serializeInt(message.messageId, send_data);
		serializeFloat(data, send_data + 4);

//		chassis_can_send_data[0] = (*(long*) &data & 0xff000000) >> 24;
//		chassis_can_send_data[1] = (*(long*) &data & 0x00ff0000) >> 16;
//		chassis_can_send_data[2] = (*(long*) &data & 0x0000ff00) >> 8;
//		chassis_can_send_data[3] = (*(long*) &data & 0x000000ff);
//		chassis_can_send_data[4] = (message.messageId & 0xff000000) >> 24;
//		chassis_can_send_data[5] = (message.messageId & 0x00ff0000) >> 16;
//		chassis_can_send_data[6] = (message.messageId & 0x0000ff00) >> 8;
//		chassis_can_send_data[7] = (message.messageId & 0x000000ff);
//	
    HAL_CAN_AddTxMessage(&MANIFOLD_CAN, &testMsgHeader, (uint8_t*) &send_data,
                       &canTxMailbox);
}

