#include "bsp_adc.h"
#include "main.h"
#include "struct_typedef.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

uint16_t get_power_5v_adc(void) {

  ADC_ChannelConfTypeDef ADC3_ChanConf;

  ADC3_ChanConf.Channel = ADC_CHANNEL_14;
  ADC3_ChanConf.Rank = 1;
  ADC3_ChanConf.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  ADC3_ChanConf.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc3, &ADC3_ChanConf) != HAL_OK) {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 10);

  return HAL_ADC_GetValue(&hadc3);
}

uint16_t get_handware_adc(void) {

  ADC_ChannelConfTypeDef ADC3_ChanConf;

  ADC3_ChanConf.Channel = ADC_CHANNEL_15;
  ADC3_ChanConf.Rank = 1;
  ADC3_ChanConf.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  ADC3_ChanConf.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc3, &ADC3_ChanConf) != HAL_OK) {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 10);

  return HAL_ADC_GetValue(&hadc3);
}

fp32 get_temprate(void) {
  fp32 temperate;
  ADC_ChannelConfTypeDef ADC1_ChanConf;

  ADC1_ChanConf.Channel = ADC_CHANNEL_TEMPSENSOR;
  ADC1_ChanConf.Rank = 1;
  ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  ADC1_ChanConf.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &ADC1_ChanConf) != HAL_OK) {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);

  temperate = (fp32)HAL_ADC_GetValue(&hadc1) * (3.3f / 4096.0f);
  temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
  return temperate;
}
