#include "bsp_buzzer.h"
#include "struct_typedef.h"
#include "main.h"




void buzzer_on(uint16_t psc, uint16_t pwm)
{
    TIM12->PSC = psc;
    TIM12->CCR1 = pwm;
}
void buzzer_off(void)
{
    TIM12->CCR1 = 0;
}

