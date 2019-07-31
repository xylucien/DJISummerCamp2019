#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define CANMESSAGE_MAX_SIZE 16
#define CANMESSAGE_ID_SIZE sizeof(uint8_t)

#define CANMESSAGE_ID_TEST 1
#define CANMESSAGE_ID_TEST_MSG_SIZE sizeof(float)

typedef struct CANMessage {
    uint8_t messageId;
    void* data;
} CANMessage;

#ifdef __cplusplus
}
#endif

#endif
