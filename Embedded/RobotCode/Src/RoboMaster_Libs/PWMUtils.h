#ifndef PWMUTILS_H
#define PWMUTILS_H

#include "stm32f4xx_hal.h"

#define PWM_FREQUENCE 50
#define PWM_RESOLUTION 10000
#define APB1_TIMER_CLOCKS 84000000
#define PWM_DEFAULT_DUTY 5000

extern void PWM_SetDuty(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty);

#endif
