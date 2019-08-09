#include "MecanumKinematics.h"
#include "MathUtils.h"
#include <math.h>
#include <stddef.h>

// ab = a + b
void mecanumKinematics(struct MecanumWheelValues *wheelValues, float ab,
                       struct Twist2D *outVector) {
  if (outVector == NULL || wheelValues == NULL) {
    return;
  }

  outVector->vX = (-wheelValues->topRight + wheelValues->topLeft +
                   wheelValues->backLeft - wheelValues->backRight) /
                  4.0f;
  outVector->vY = (-wheelValues->topRight - wheelValues->topLeft +
                   wheelValues->backLeft + wheelValues->backRight) /
                  4.0f;

  outVector->w = (-wheelValues->topRight - wheelValues->topLeft -
                  wheelValues->backLeft - wheelValues->backRight) /
                 (4.0f * ab);
}

void mecanumInverseKinematics(struct Twist2D *targetVelocity, float ab, float r,
                              struct MecanumWheelValues *outputWheels) {
  if (outputWheels == NULL || targetVelocity == NULL) {
    return;
  }

  outputWheels->topRight =
      (targetVelocity->vY - targetVelocity->w * ab - targetVelocity->vX) / r;
  outputWheels->topLeft =
      (targetVelocity->vY + targetVelocity->w * ab + targetVelocity->vX ) / r;
  outputWheels->backLeft =
      (targetVelocity->vY + targetVelocity->w * ab - targetVelocity->vX ) / r;
  outputWheels->backRight =
      (targetVelocity->vY - targetVelocity->w * ab + targetVelocity->vX ) / r;
}

// TODO Not finished, needs to rotate vx and vy
void integrateVelocities(struct MecanumWheelValues *newVelocity, float ab,
                         float currentTime, float currentYaw,
                         struct MecanumPosition *position) {
  if (newVelocity == NULL || position == NULL) {
    return;
  }

  double dt = (currentTime - position->lastRanTime);

  if (dt < 0.0) {
    return;
  }

  position->lastRanTime = currentTime;

  if(newVelocity->topLeft == 0 && newVelocity->topRight == 0 && newVelocity->backLeft == 0 && newVelocity->backRight){
    return;
  }

  Twist2D velocityVector;
  mecanumKinematics(newVelocity, ab, &velocityVector);
  rotateTwist2D(-currentYaw, &velocityVector, &velocityVector);

  position->x += velocityVector.vX * dt;
  position->y += velocityVector.vY * dt;

  //Maybe use gyro yaw
  position->yaw += velocityVector.w * dt;

}

// Gyro yaw in radians
void transformInputToFieldOriented(Twist2D *joystickInput, float currentYaw) {
  if (joystickInput == NULL) {
    return;
  }

  float cosYaw = cosf(currentYaw);
  float sinYaw = sinf(currentYaw);

  float calcTemp = joystickInput->vY * cosYaw + joystickInput->vX * sinYaw;

  joystickInput->vX = -joystickInput->vY * sinYaw + joystickInput->vX * cosYaw;
  joystickInput->vY = calcTemp;
}
