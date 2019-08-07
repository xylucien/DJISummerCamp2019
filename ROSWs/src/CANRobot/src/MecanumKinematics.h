#include "MathUtils.h"

typedef struct MecanumWheelValues {
  float topLeft;
  float topRight;

  float backLeft;
  float backRight;
} MecanumWheelValues;

typedef struct MecanumPosition {
  float x;
  float y;

  float yaw;

  // Still need to reliably get time between runs assume a float for now
  float lastRanTime;
} MecanumPosition;

extern void mecanumKinematics(MecanumWheelValues *, float, Twist2D *);

extern void mecanumInverseKinematics(Twist2D *, float, float,
                                     MecanumWheelValues *);

extern void integrateVelocities(MecanumWheelValues *, float, float, float,
                                MecanumPosition *position);

extern void transformInputToFieldOriented(Twist2D *, float);
