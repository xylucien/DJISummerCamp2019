#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#ifdef __cplusplus
extern "C"
{
#endif

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
#define CANMESSAGE_SUBID_ODOM_X 0
#define CANMESSAGE_SUBID_ODOM_Y 1
#define CANMESSAGE_SUBID_ODOM_YAW 2

#define CANMESSAGE_ID_PWM 3
#define CANMESSAGE_SUBID_PWM0 0

#define CANMESSAGE_ID_BUZZER 4
#define CANMESSAGE_SUBID_BUZZER_DUTYCYCLE 0
#define CANMESSAGE_SUBID_BUZZER_FREQUENCY 1
#define CANMESSAGE_SUBID_BUZZER_OFF 2

#define CANMESSAGE_ID_AHRS 5
#define CANMESSAGE_SUBID_AHRS_ROLL 0
#define CANMESSAGE_SUBID_AHRS_PITCH 1
#define CANMESSAGE_SUBID_AHRS_YAW 2

#define CANMESSAGE_ID_MECANISIM 6
#define CANMESSAGE_SUBID_RIGHT_BALL_POSITION 0
#define CANMESSAGE_SUBID_LEFT_BALL_POSITION 1

//No fancy IDs here
typedef struct RawCANMessage {
    uint32_t id;
    void* data;
} RawCANMessage;

typedef struct CANId {
    uint32_t baseId;
    uint8_t messageId;
    uint8_t subId;
} CANId;

typedef struct CANMessage {
    CANId id;
    void* data;
} CANMessage;

#ifdef __cplusplus
}
#endif

#endif
