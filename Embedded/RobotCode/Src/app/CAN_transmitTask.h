#ifndef CAN_TRANSMITTASK_H
#define CAN_TRANSMITTASK_H

#define MANIFOLD_CAN hcan2

#include <stdint.h>

extern void canTransmitTaskLoop(void const *argument);
extern void canSendTestMessage(void);

extern void canSendFloatMessage(uint8_t id, uint8_t subid, float data);

#endif
