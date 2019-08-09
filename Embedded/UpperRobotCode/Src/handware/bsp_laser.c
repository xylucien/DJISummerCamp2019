#include "bsp_laser.h"
#include "main.h"
#include "struct_typedef.h"

void laser_on(void) {
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}
void laser_off(void) {
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}
