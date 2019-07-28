#pragma once

typedef struct Twist2D {
  float vX;
  float vY;
  float w;
} Twist2D;

extern void rotateTwist2D(float, Twist2D*, Twist2D*);
