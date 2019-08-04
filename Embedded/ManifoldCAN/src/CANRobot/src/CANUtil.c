#include "CANUtil.h"
#include <string.h>

void serializeFloat(float input, uint8_t *dest){
    memcpy(dest, &input, sizeof(float));

    // dest[0] = (*(long*) &input & 0xff000000) >> 24;
    // dest[1] = (*(long*) &input & 0x00ff0000) >> 16;
    // dest[2] = (*(long*) &input & 0x0000ff00) >> 8;
    // dest[3] = (*(long*) &input & 0x000000ff);
}

float deserializeFloat(uint8_t *source){
    float output;

    memcpy(&output, source, sizeof(float));
    return output;

    // long buffer = 0;
    // buffer |= (source[0]) << 24;
    // buffer |= (source[1]) << 16;
    // buffer |= (source[2]) << 8;
    // buffer |= (source[3]);
    // return *(float*) &buffer;
}

void serializeInt(uint8_t input, uint8_t *dest){
    memcpy(dest, &input, sizeof(uint8_t));

    //dest[0] = (*(long*) &input & 0xff000000) >> 24;
    //dest[1] = (*(long*) &input & 0x00ff0000) >> 16;
    //dest[2] = (*(long*) &input & 0x0000ff00) >> 8;
    //dest[3] = (*(long*) &input & 0x000000ff);
}

uint8_t deserializeInt(uint8_t *source){
    uint8_t output;
    memcpy(&output, source, sizeof(uint8_t));
    return output;

    // long buffer = 0;
    // buffer |= (source[0]) << 24;
    // buffer |= (source[1]) << 16;
    // buffer |= (source[2]) << 8;
    // buffer |= (source[3]);
    // return *(uint8_t*) &buffer;
}
