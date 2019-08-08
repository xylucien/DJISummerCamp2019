#include "mecanisimCANTXTask.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "string.h"
#include "stdlib.h"

extern CAN_HandleTypeDef hcan2;

int16_t motor8Set = 0.0;
int16_t motor9Set = 0.0;
int16_t motor10Set = 0.0;
int16_t motor11Set = 0.0;
int16_t motor12Set = 0.0;
int16_t motor13Set = 0.0;
int16_t motor14Set = 0.0;
int16_t motor15Set = 0.0;

bool new8_11data = true;
bool new12_15data = true;

void mecanisimCANTXTaskUpdate(void *arguments){
    for(;;){
        uint32_t send_mailBox;

        static CAN_TxHeaderTypeDef txCANHeader;
        static uint8_t txMessage[8];

		
        //First batch (8-11) @0x800 -> (0-3) @ 0x700
		if(new8_11data){
			txCANHeader.StdId = CAN_CHASSIS_ALL_ID;
			txCANHeader.IDE = CAN_ID_STD;
			txCANHeader.RTR = CAN_RTR_DATA;
			txCANHeader.DLC = 0x08;
			
			//8
			txMessage[0] = motor8Set >> 8;
			txMessage[1] = motor8Set;
			//9
			txMessage[2] = motor9Set >> 8;
			txMessage[3] = motor9Set;
			//10
			txMessage[4] = motor10Set >> 8;
			txMessage[5] = motor10Set;
			//11
			txMessage[6] = motor11Set >> 8;
			txMessage[7] = motor11Set;

			HAL_CAN_AddTxMessage(&hcan2, &txCANHeader, txMessage,
							&send_mailBox);
			
			new8_11data = false;
		}
        

		if(new12_15data){
			txCANHeader.StdId = 0x1FF; //First batch (12-15) @0x801 -> (4-7) @ 0x1FF
			txCANHeader.IDE = CAN_ID_STD;
        	txCANHeader.RTR = CAN_RTR_DATA;
        	txCANHeader.DLC = 0x08;
        	//12
			txMessage[0] = motor12Set >> 8;
			txMessage[1] = motor12Set;
			//13
			txMessage[2] = motor13Set >> 8;
			txMessage[3] = motor13Set;
			//14
			txMessage[4] = motor14Set >> 8;
			txMessage[5] = motor14Set;
			//15
			txMessage[6] = motor15Set >> 8;
			txMessage[7] = motor15Set;

        	HAL_CAN_AddTxMessage(&hcan2, &txCANHeader, txMessage,
                        &send_mailBox);

			new12_15data = false;
		}
        

        vTaskDelay(25);
    }
}
