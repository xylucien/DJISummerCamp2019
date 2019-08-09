#include "mecanisimCANTXTask.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "string.h"
#include "stdlib.h"

extern CAN_HandleTypeDef hcan2;

uint16_t motor8Set = 0.0;
uint16_t motor9Set = 0.0;
uint16_t motor10Set = 0.0;
uint16_t motor11Set = 0.0;
uint16_t motor12Set = 0.0;
uint16_t motor13Set = 0.0;
uint16_t motor14Set = 0.0;
uint16_t motor15Set = 0.0;

void mecanisimCANTXTaskUpdate(void *arguments){
    for(;;){
        uint32_t send_mailBox;

        static CAN_TxHeaderTypeDef txCANHeader;
        static uint8_t txMessage[8];

        //First batch (8-11) @0x800 -> (0-3) @ 0x700
        txCANHeader.StdId = CAN_CHASSIS_ALL_ID;
        txCANHeader.IDE = CAN_ID_STD;
        txCANHeader.RTR = CAN_RTR_DATA;
        txCANHeader.DLC = 0x08;
			
				//8
				txMessage[0] = motor8Set >> 8;
				txMessage[1] = motor8Set;
				//9
				//txMessage[2] = rx_data[3];
				//txMessage[3] = rx_data[2];
				//10
				//txMessage[4] = rx_data[5];
				//txMessage[5] = rx_data[4];
				//11
				//txMessage[6] = rx_data[7];
				//txMessage[7] = rx_data[6];

        HAL_CAN_AddTxMessage(&hcan2, &txCANHeader, txMessage,
                        &send_mailBox);

        txCANHeader.StdId = 0x801; //First batch (12-15) @0x801 -> (4-7) @ 0x1FF
        memcpy(txMessage, &motor12Set, sizeof(uint16_t));
        memcpy(&(txMessage[2]), &motor13Set, sizeof(uint16_t));
        memcpy(&txMessage[4], &motor14Set, sizeof(uint16_t));
        memcpy(&txMessage[6], &motor15Set, sizeof(uint16_t));

        HAL_CAN_AddTxMessage(&hcan2, &txCANHeader, txMessage,
                        &send_mailBox);

        vTaskDelay(1);
    }
}
