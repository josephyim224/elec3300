/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define true 1
#define false 0
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOC
#define BATT_Pin GPIO_PIN_1
#define BATT_GPIO_Port GPIOA
#define M0_NTC_Pin GPIO_PIN_4
#define M0_NTC_GPIO_Port GPIOA
#define M1_NTC_Pin GPIO_PIN_5
#define M1_NTC_GPIO_Port GPIOA
#define M2_NTC_Pin GPIO_PIN_6
#define M2_NTC_GPIO_Port GPIOA
#define M1_VPROPI_Pin GPIO_PIN_7
#define M1_VPROPI_GPIO_Port GPIOA
#define M2_VPROPI_Pin GPIO_PIN_0
#define M2_VPROPI_GPIO_Port GPIOB
#define M0_VPROPI_Pin GPIO_PIN_1
#define M0_VPROPI_GPIO_Port GPIOB
#define M1_nFAULT_Pin GPIO_PIN_2
#define M1_nFAULT_GPIO_Port GPIOB
#define M2_nFAULT_Pin GPIO_PIN_10
#define M2_nFAULT_GPIO_Port GPIOB
#define M0_nFAULT_Pin GPIO_PIN_11
#define M0_nFAULT_GPIO_Port GPIOB
#define M_MODE1_Pin GPIO_PIN_12
#define M_MODE1_GPIO_Port GPIOB
#define M_nSLEEP_Pin GPIO_PIN_13
#define M_nSLEEP_GPIO_Port GPIOB
#define M1_ENABLE_Pin GPIO_PIN_14
#define M1_ENABLE_GPIO_Port GPIOB
#define M2_ENABLE_Pin GPIO_PIN_15
#define M2_ENABLE_GPIO_Port GPIOB
#define M1_PHASE_Pin GPIO_PIN_8
#define M1_PHASE_GPIO_Port GPIOA
#define M2_PHASE_Pin GPIO_PIN_9
#define M2_PHASE_GPIO_Port GPIOA
#define M0_PHASE_Pin GPIO_PIN_10
#define M0_PHASE_GPIO_Port GPIOA
#define M0_ENABLE_Pin GPIO_PIN_11
#define M0_ENABLE_GPIO_Port GPIOA
#define M_MODE0_Pin GPIO_PIN_12
#define M_MODE0_GPIO_Port GPIOA
#define M0_ENC_Pin GPIO_PIN_15
#define M0_ENC_GPIO_Port GPIOA
#define M2_ENC_Pin GPIO_PIN_4
#define M2_ENC_GPIO_Port GPIOB
#define WS2812B_Pin GPIO_PIN_5
#define WS2812B_GPIO_Port GPIOB
#define M1_ENC_Pin GPIO_PIN_6
#define M1_ENC_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
