#include "bsp_power_ctrl.h"
#include "main.h"
#include "struct_typedef.h"

void power_ctrl_on(uint8_t num) {
  if (num > POWER4_CTRL_SWITCH) {
    return;
  }
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2 << num, GPIO_PIN_SET);
}

void power_ctrl_off(uint8_t num) {
  if (num > POWER4_CTRL_SWITCH) {
    return;
  }
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2 << num, GPIO_PIN_RESET);
}

void power_ctrl_toggle(uint8_t num) {
  if (num > POWER4_CTRL_SWITCH) {
    return;
  }
  HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_2 << num);
}
