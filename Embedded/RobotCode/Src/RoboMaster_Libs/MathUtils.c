#include "MathUtils.h"
#include <stddef.h>
#include <math.h>

void rotateTwist2D(float rotation, Twist2D* twistToRotate,
                   Twist2D* rotatedTwist) {
  if (twistToRotate == NULL || rotatedTwist == NULL) {
    return;
  }

  float cosA = cosf(rotation);
  float sinA = sinf(rotation);

  rotatedTwist->vX = cosA * twistToRotate->vX - sinA * twistToRotate->vY;
  rotatedTwist->vY = sinA * twistToRotate->vX + cosA * twistToRotate->vY;
}

