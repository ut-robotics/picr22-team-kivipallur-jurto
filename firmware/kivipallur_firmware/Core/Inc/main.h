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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_B1_Pin GPIO_PIN_0
#define LED_B1_GPIO_Port GPIOF
#define LED_B2_Pin GPIO_PIN_1
#define LED_B2_GPIO_Port GPIOF
#define Debug_NRST_Pin GPIO_PIN_10
#define Debug_NRST_GPIO_Port GPIOG
#define Mikro_M1_en1_Pin GPIO_PIN_0
#define Mikro_M1_en1_GPIO_Port GPIOA
#define Mikro_M1_en2_Pin GPIO_PIN_1
#define Mikro_M1_en2_GPIO_Port GPIOA
#define Mikro_M2_en1_Pin GPIO_PIN_2
#define Mikro_M2_en1_GPIO_Port GPIOA
#define Mikro_Visk_Pin GPIO_PIN_3
#define Mikro_Visk_GPIO_Port GPIOA
#define LED_R1_Pin GPIO_PIN_4
#define LED_R1_GPIO_Port GPIOA
#define LED_R2_Pin GPIO_PIN_5
#define LED_R2_GPIO_Port GPIOA
#define Mikro_Palli_H_Pin GPIO_PIN_6
#define Mikro_Palli_H_GPIO_Port GPIOA
#define Mikro_Palli_S_Pin GPIO_PIN_7
#define Mikro_Palli_S_GPIO_Port GPIOA
#define Mikro_M_sleep_Pin GPIO_PIN_0
#define Mikro_M_sleep_GPIO_Port GPIOB
#define Mikro_M3_en2_Pin GPIO_PIN_8
#define Mikro_M3_en2_GPIO_Port GPIOA
#define Mikro_M3_en1_Pin GPIO_PIN_9
#define Mikro_M3_en1_GPIO_Port GPIOA
#define Mikro_M2_en2_Pin GPIO_PIN_10
#define Mikro_M2_en2_GPIO_Port GPIOA
#define Mikro_m3_en1_Pin GPIO_PIN_15
#define Mikro_m3_en1_GPIO_Port GPIOA
#define Mikro_M_off_Pin GPIO_PIN_3
#define Mikro_M_off_GPIO_Port GPIOB
#define Mikro_m2_en1_Pin GPIO_PIN_4
#define Mikro_m2_en1_GPIO_Port GPIOB
#define Mikro_m2_en2_Pin GPIO_PIN_5
#define Mikro_m2_en2_GPIO_Port GPIOB
#define Mikro_m1_en1_Pin GPIO_PIN_6
#define Mikro_m1_en1_GPIO_Port GPIOB
#define Mikro_m1_en2_Pin GPIO_PIN_7
#define Mikro_m1_en2_GPIO_Port GPIOB
#define Mikro_m3_en2_Pin GPIO_PIN_8
#define Mikro_m3_en2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
