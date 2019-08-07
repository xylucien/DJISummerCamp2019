/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define Latitude_At_ShenZhen 22.57025f
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IST8310_INT_Pin GPIO_PIN_3
#define IST8310_INT_GPIO_Port GPIOE
#define IST8310_INT_EXTI_IRQn EXTI3_IRQn
#define IST8310_RST_Pin GPIO_PIN_2
#define IST8310_RST_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define SPI4_NSS_Pin GPIO_PIN_4
#define SPI4_NSS_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define FLOW_LED_8_Pin GPIO_PIN_8
#define FLOW_LED_8_GPIO_Port GPIOG
#define POWER5V_AD_Pin GPIO_PIN_4
#define POWER5V_AD_GPIO_Port GPIOF
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define FLOW_LED_7_Pin GPIO_PIN_7
#define FLOW_LED_7_GPIO_Port GPIOG
#define FLOW_LED_6_Pin GPIO_PIN_6
#define FLOW_LED_6_GPIO_Port GPIOG
#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF
#define HW_VC_AD_Pin GPIO_PIN_5
#define HW_VC_AD_GPIO_Port GPIOF
#define FLOW_LED_5_Pin GPIO_PIN_5
#define FLOW_LED_5_GPIO_Port GPIOG
#define FLOW_LED_4_Pin GPIO_PIN_4
#define FLOW_LED_4_GPIO_Port GPIOG
#define FLOW_LED_3_Pin GPIO_PIN_3
#define FLOW_LED_3_GPIO_Port GPIOG
#define SHOOT_INPUT_Pin GPIO_PIN_10
#define SHOOT_INPUT_GPIO_Port GPIOF
#define FLOW_LED_2_Pin GPIO_PIN_2
#define FLOW_LED_2_GPIO_Port GPIOG
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define FLOW_LED_1_Pin GPIO_PIN_1
#define FLOW_LED_1_GPIO_Port GPIOG
#define BUTTON_AD_Pin GPIO_PIN_6
#define BUTTON_AD_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define SD_EXTI_Pin GPIO_PIN_15
#define SD_EXTI_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
