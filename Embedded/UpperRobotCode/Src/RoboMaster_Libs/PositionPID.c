#include "PositionPID.h"

void initializePositionPid(PositionPIDData *data){
    if(data == NULL || data->velocityPid == NULL || data->positionPid == NULL){
        return;
    }
    
    arm_pid_init_f32(data->velocityPid, 1);
    arm_pid_init_f32(data->positionPid, 1);
}

float32_t calculatePositionPid(PositionPIDData* data, float32_t currentVelocity, float32_t currentPosition, float32_t positionSetPoint){
    if(data->positionLimitEnabled){
        if(positionSetPoint > data->maximumPosition)
            positionSetPoint = data->maximumPosition;
        else if(positionSetPoint < data->minimumPosition)
            positionSetPoint = data->minimumPosition;
    }
    
    float32_t positionError = positionSetPoint - currentPosition;
    float32_t velocitySet = arm_pid_f32(data->positionPid, positionError);

    if(velocitySet > data->maximumVelocity){
        velocitySet = data->maximumVelocity;
    } else if(velocitySet < -data->maximumVelocity){
        velocitySet = -data->maximumVelocity;
    }
		
		data->previousVelocitySet = velocitySet;

    float32_t velocityError = velocitySet - currentVelocity;
    float32_t setPoint = arm_pid_f32(data->velocityPid, velocityError);
		
		if(setPoint > data->maximumOutput){
			setPoint = data->maximumOutput;
		} else if (setPoint < -data->maximumOutput){
			setPoint = -data->maximumOutput;
		}
		
		return setPoint;
}
