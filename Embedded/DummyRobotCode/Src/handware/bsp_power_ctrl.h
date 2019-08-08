#ifndef BSP_POWER_CTRL_H
#define BSP_POWER_CTRL_H

#include "struct_typedef.h"

#define POWER1_CTRL_SWITCH 0
#define POWER2_CTRL_SWITCH 1
#define POWER3_CTRL_SWITCH 2
#define POWER4_CTRL_SWITCH 3

extern void power_ctrl_on(uint8_t num);
extern void power_ctrl_off(uint8_t num);
extern void power_ctrl_toggle(uint8_t num);

#endif
