#include <stdint.h>
#include "CANMessage.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern void serializeFloat(float input, uint8_t *dest);
extern float deserializeFloat(uint8_t *source);

extern void serializeInt(uint8_t input, uint8_t *dest);
extern uint8_t deserializeInt(uint8_t *source);

extern uint32_t serializeId(CANId id);
extern CANId deserializeId(uint32_t canId);

#ifdef __cplusplus
}
#endif