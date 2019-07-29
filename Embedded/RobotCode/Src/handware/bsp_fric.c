#include "bsp_fric.h"
#include "main.h"
#include "struct_typedef.h"

void fric_off(void) {
  TIM1->CCR1 = Fric_OFF;
  TIM1->CCR4 = Fric_OFF;
}
void fric1_on(uint16_t cmd) { TIM1->CCR1 = cmd; }
void fric2_on(uint16_t cmd) { TIM1->CCR4 = cmd; }
