#include "bsp_rng.h"
#include "main.h"
#include "struct_typedef.h"

extern RNG_HandleTypeDef hrng;

uint32_t RNG_get_random_num(void) {
  uint32_t num;
  HAL_RNG_GenerateRandomNumber(&hrng, &num);
  return num;
}

int32_t RNG_get_random_range(int min, int max) {
  return RNG_get_random_num() % (max - min + 1) + min;
}
