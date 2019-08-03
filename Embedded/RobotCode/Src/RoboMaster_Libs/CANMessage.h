#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#include <stdint.h>

#define CANMESSAGE_MAX_SIZE 16
#define CANMESSAGE_ID_SIZE sizeof(uint8_t)

#define CANMESSAGE_MANIFOLD_BASE_ID 0x600

//Test message
#define CANMESSAGE_ID_TEST 1
#define CANMESSAGE_ID_TEST_MSG_SIZE sizeof(float)

//Target Velocity (floats)
#define CANMESSAGE_ID_TARGET_VEL 1
#define CANMESSAGE_SUBID_TARGET_VX 0
#define CANMESSAGE_SUBID_TARGET_VY 1
#define CANMESSAGE_SUBID_TARGET_VW 2
#define CANMESSAGE_SUBID_TARGET_READY 3

#define CANMESSAGE_ID_ODOMETRY 2
#define CANMESSAGE_SUBID_ODOMETRY_X 0
#define CANMESSAGE_SUBID_ODOMETRY_Y 1
#define CANMESSAGE_SUBID_ODOMETRY_W 2
#define CANMESSAGE_SUBID_ODOMETRY_READY 3

typedef struct CANMessage {
    uint8_t messageId;
    void* data;
} CANMessage;

#endif
