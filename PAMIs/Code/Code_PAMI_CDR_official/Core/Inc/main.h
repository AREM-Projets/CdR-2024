/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define DIS_GAUCHE_Pin GPIO_PIN_1
#define DIS_GAUCHE_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define DIS_DROITE_Pin GPIO_PIN_3
#define DIS_DROITE_GPIO_Port GPIOA
#define ECHO_Pin GPIO_PIN_0
#define ECHO_GPIO_Port GPIOB
#define TRIGGER_Pin GPIO_PIN_1
#define TRIGGER_GPIO_Port GPIOB
#define DIR_GAUCHE_Pin GPIO_PIN_8
#define DIR_GAUCHE_GPIO_Port GPIOA
#define PWM_DROITE_Pin GPIO_PIN_9
#define PWM_DROITE_GPIO_Port GPIOA
#define PWM_GAUCHE_Pin GPIO_PIN_10
#define PWM_GAUCHE_GPIO_Port GPIOA
#define DIR_DROTE_Pin GPIO_PIN_11
#define DIR_DROTE_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define Tirette_Pin GPIO_PIN_4
#define Tirette_GPIO_Port GPIOB
#define SW_Pin GPIO_PIN_5
#define SW_GPIO_Port GPIOB
#define SW_EXTI_IRQn EXTI9_5_IRQn
#define LED_JAUNE_Pin GPIO_PIN_6
#define LED_JAUNE_GPIO_Port GPIOB
#define LED_BLEUE_Pin GPIO_PIN_7
#define LED_BLEUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
