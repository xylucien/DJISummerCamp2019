#ifndef BSP_LED_H
#define BSP_LED_H

#include "struct_typedef.h"

extern void led_green_off(void);
extern void led_green_on(void);
extern void led_green_toggle(void);
extern void led_red_off(void);
extern void led_red_on(void);
extern void led_red_toggle(void);
extern void flow_led_on(uint16_t num);
extern void flow_led_off(uint16_t num);
extern void flow_led_toggle(uint16_t num);

#endif



