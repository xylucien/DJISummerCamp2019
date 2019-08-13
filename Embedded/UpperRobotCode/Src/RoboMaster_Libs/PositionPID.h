#ifndef POSITION_PID_H
#define POSITION_PID_H

#include <stdbool.h>
#include <arm_math.h>

typedef struct PositionPIDData {
    arm_pid_instance_f32 *velocityPid;
    float32_t maximumVelocity;

    arm_pid_instance_f32 *positionPid;
    bool positionLimitEnabled;
    float32_t maximumPosition;
    float32_t minimumPosition;
	
		float32_t maximumOutput;
	
		float32_t previousVelocitySet;
	
} PositionPIDData;

extern void initializePositionPid(PositionPIDData *data);
extern float32_t calculatePositionPid(PositionPIDData *data, float32_t currentVelocity, float32_t currentPosition, float32_t positionSetPoint);

#endif
