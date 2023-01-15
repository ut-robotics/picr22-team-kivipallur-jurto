/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define LED_B1_D3_Pin GPIO_PIN_0
#define LED_B1_D3_GPIO_Port GPIOF
#define LED_B2_Pin GPIO_PIN_1
#define LED_B2_GPIO_Port GPIOF
#define MOT2_PWM_1_Pin GPIO_PIN_0
#define MOT2_PWM_1_GPIO_Port GPIOA
#define MOT2_suund_Pin GPIO_PIN_1
#define MOT2_suund_GPIO_Port GPIOA
#define MOT3_suund_Pin GPIO_PIN_2
#define MOT3_suund_GPIO_Port GPIOA
#define TRW_PWM_1_Pin GPIO_PIN_3
#define TRW_PWM_1_GPIO_Port GPIOA
#define LED_R1_D1_Pin GPIO_PIN_4
#define LED_R1_D1_GPIO_Port GPIOA
#define LED_R2_D2_Pin GPIO_PIN_5
#define LED_R2_D2_GPIO_Port GPIOA
#define LED_R2_D2_EXTI_IRQn EXTI9_5_IRQn
#define HOLD_servo_PWM_Pin GPIO_PIN_6
#define HOLD_servo_PWM_GPIO_Port GPIOA
#define AIM_servo_PWM_Pin GPIO_PIN_7
#define AIM_servo_PWM_GPIO_Port GPIOA
#define MOT_SLEEP_Pin GPIO_PIN_0
#define MOT_SLEEP_GPIO_Port GPIOB
#define MOT1_suund_Pin GPIO_PIN_8
#define MOT1_suund_GPIO_Port GPIOA
#define MOT1_PWM_2_Pin GPIO_PIN_9
#define MOT1_PWM_2_GPIO_Port GPIOA
#define MOT3_PWM_2_Pin GPIO_PIN_10
#define MOT3_PWM_2_GPIO_Port GPIOA
#define MOT3_ENC_1_Pin GPIO_PIN_15
#define MOT3_ENC_1_GPIO_Port GPIOA
#define MOT_OFF_Pin GPIO_PIN_3
#define MOT_OFF_GPIO_Port GPIOB
#define MOT1_ENC_1_Pin GPIO_PIN_4
#define MOT1_ENC_1_GPIO_Port GPIOB
#define MOT1_ENC_2_Pin GPIO_PIN_5
#define MOT1_ENC_2_GPIO_Port GPIOB
#define MOT2_ENC_1_Pin GPIO_PIN_6
#define MOT2_ENC_1_GPIO_Port GPIOB
#define MOT2_ENC_2_Pin GPIO_PIN_7
#define MOT2_ENC_2_GPIO_Port GPIOB
#define MOT3_ENC_2_Pin GPIO_PIN_8
#define MOT3_ENC_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
