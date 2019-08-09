#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <AHRS_middleware.h>
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "MecanumKinematics.h"

//((codes / 8191) / 3591) * 187 * wheelCircumference
//#define ENCODERCODES_TO_MS 8.09699811693132299769491208890938e-7f
#define ENCODERCODES_TO_MS 3.0458533676241814e-06f

extern void initChassisOdom(void);
extern void chassisOdomUpdate(void const *argument);


