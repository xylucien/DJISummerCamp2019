#include "CANUtil.h"
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

uint32_t serializeId(CANId id){
    uint32_t output = id.baseId;
    output |= (id.messageId << 4);
    output |= id.subId;

    return output;
}

#include <stdio.h>

CANId deserializeId(uint32_t canId){
    CANId output;
    output.baseId = (canId & 0x00000F00);
    output.messageId = (uint8_t) (canId & 0x000000F0) >> 4;
    output.subId = (uint8_t) (canId & 0x0000000F);

    return output;
}
