#include <math.h>
#include <stddef.h>
#include "MecanumKinematics.h"
#include "MathUtils.h"

//ab = a + b
void mecanumKinematics(struct MecanumWheelValues *wheelValues, float ab, struct Twist2D *outVector){
    if(outVector == NULL || wheelValues == NULL){
        return;
    }
    
    outVector->vX = (-wheelValues->topRight + wheelValues->topLeft + wheelValues->backLeft - wheelValues->backRight) / 4.0f;
    outVector->vY = (-wheelValues->topRight - wheelValues->topLeft + wheelValues->backLeft + wheelValues->backRight) / 4.0f;

    outVector->w = (-wheelValues->topRight - wheelValues->topLeft - wheelValues->backLeft - wheelValues->backRight) / (4.0f * ab);
}

void mecanumInverseKinematics(struct Twist2D *targetVelocity, float ab, float r, struct MecanumWheelValues *outputWheels){
    if(outputWheels == NULL || targetVelocity == NULL){
        return;
    }
    
    outputWheels->topRight = (targetVelocity->vX - targetVelocity->vY - targetVelocity->w * ab) / r;
    outputWheels->topLeft = (targetVelocity->vX + targetVelocity->vY + targetVelocity->w * ab) / r;
    outputWheels->backLeft = (targetVelocity->vX + targetVelocity->vY - targetVelocity->w * ab) / r;
    outputWheels->backRight = (targetVelocity->vX - targetVelocity->vY + targetVelocity->w * ab) / r;
}

//TODO Not finished, needs to rotate vx and vy
void integrateVelocities(struct MecanumWheelValues *newVelocity, float ab, float currentTime, float currentYaw, struct MecanumPosition *position){
    if(newVelocity == NULL || position == NULL){
        return;
    }

    double dt = currentTime - position->lastRanTime;

    if(dt < 0.0){
        return;
    }

    Twist2D velocityVector;

    rotateTwist2D(-currentYaw, &velocityVector, &velocityVector);

    mecanumKinematics(newVelocity, ab,& velocityVector);

    float dX = velocityVector.vX * dt;
    float dY = velocityVector.vY * dt;

    float dW = velocityVector.w * dt;
	}

//Gyro yaw in radians
void transformInputToFieldOriented(Twist2D *joystickInput, float currentYaw){
    if(joystickInput == NULL){
        return;
    }

    float cosYaw = cosf(currentYaw);
    float sinYaw = sinf(currentYaw);

    float calcTemp = joystickInput->vY * cosYaw + joystickInput->vX * sinYaw;

    joystickInput->vX = -joystickInput->vY * sinYaw + joystickInput->vX * cosYaw;
    joystickInput->vY = calcTemp;
}
