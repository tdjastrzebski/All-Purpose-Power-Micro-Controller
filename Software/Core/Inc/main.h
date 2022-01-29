/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define TIMERS_CLOCK_FREQ 16000000
#define DC_Pin GPIO_PIN_13
#define DC_GPIO_Port GPIOC
#define ADC_A_Pin GPIO_PIN_0
#define ADC_A_GPIO_Port GPIOA
#define ADC_B_Pin GPIO_PIN_1
#define ADC_B_GPIO_Port GPIOA
#define DAC_A_Pin GPIO_PIN_4
#define DAC_A_GPIO_Port GPIOA
#define DAC_B_Pin GPIO_PIN_5
#define DAC_B_GPIO_Port GPIOA
#define SDA_B_Pin GPIO_PIN_6
#define SDA_B_GPIO_Port GPIOA
#define SCL_B_Pin GPIO_PIN_7
#define SCL_B_GPIO_Port GPIOA
#define TX_B_Pin GPIO_PIN_0
#define TX_B_GPIO_Port GPIOB
#define RX_B_Pin GPIO_PIN_1
#define RX_B_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define SCL_A_Pin GPIO_PIN_13
#define SCL_A_GPIO_Port GPIOB
#define SDA_A_Pin GPIO_PIN_14
#define SDA_A_GPIO_Port GPIOB
#define BTN_Pin GPIO_PIN_8
#define BTN_GPIO_Port GPIOA
#define BTN_EXTI_IRQn EXTI4_15_IRQn
#define TX_A_Pin GPIO_PIN_9
#define TX_A_GPIO_Port GPIOA
#define ENC2_Pin GPIO_PIN_6
#define ENC2_GPIO_Port GPIOC
#define ENC1_Pin GPIO_PIN_7
#define ENC1_GPIO_Port GPIOC
#define RX_A_Pin GPIO_PIN_10
#define RX_A_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_3
#define PWM_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_8
#define SCK_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_9
#define CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
