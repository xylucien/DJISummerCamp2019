#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

extern void serializeFloat(float input, uint8_t *dest);
extern float deserializeFloat(uint8_t *source);

extern void serializeInt(uint8_t input, uint8_t *dest);
extern uint8_t deserializeInt(uint8_t *source);

#ifdef __cplusplus
}
#endif
