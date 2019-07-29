#ifndef BSP_FRIC_H
#define BSP_FRIC_H

#include "struct_typedef.h"

#define Fric_UP 1400
#define Fric_DOWN 1300
#define Fric_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
