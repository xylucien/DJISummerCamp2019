#include "CANUtil.h"
#include "CANMessage.h"
#include <string.h>

void serializeFloat(float input, uint8_t *dest){
    memcpy(dest, &input, sizeof(float));
}

float deserializeFloat(uint8_t *source){
    float output;

    memcpy(&output, source, sizeof(float));
    return output;
}

void serializeInt(uint8_t input, uint8_t *dest){
    memcpy(dest, &input, sizeof(uint8_t));
}

uint8_t deserializeInt(uint8_t *source){
    uint8_t output;
    memcpy(&output, source, sizeof(uint8_t));
    return output;
}

uint32_t calculateId(uint8_t canId, uint8_t subId){
    uint32_t output = CANMESSAGE_MANIFOLD_BASE_ID;
    output |= (canId << 4);
    output |= subId;

    return output;
}

