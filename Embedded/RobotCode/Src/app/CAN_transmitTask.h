#ifndef CAN_TRANSMITTASK_H
#define CAN_TRANSMITTASK_H

#define MANIFOLD_CAN hcan2

extern void canTransmitTaskLoop(void const *argument);
extern void canSendTestMessage(void);

#endif
