/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd_USB_FS;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim8_ch1;

DMA_HandleTypeDef hdma_dma_generator0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct Command { // (1)
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t throwerSpeed;
  uint16_t delimiter; // (2)
} Command;

typedef struct Feedback { // (3)
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t delimiter;
} Feedback;


typedef struct MotorControl {
	int16_t speed;
	int16_t position;
	int32_t integraal;
	int16_t p_gain;
	int16_t i_gain;
	int32_t newspeed;
	int16_t positionChange;
} MotorControl;

MotorControl motor1Control = {
	  .speed = 0,
	  .position = 0,
	  .integraal = 0,
	  .p_gain = 3000,
	  .i_gain = 675,
	  .newspeed = 0,
	  .positionChange = 0
};


MotorControl motor2Control = {
	  .speed = 0,
	  .position = 0,
	  .integraal = 0,
	  .p_gain = 3000,
	  .i_gain = 675,
	  .newspeed = 0,
	  .positionChange = 0
};

MotorControl motor3Control = {
	  .speed = 0,
	  .position = 0,
	  .integraal = 0,
	  .p_gain = 3000,
	  .i_gain = 675,
	  .newspeed = 0,
	  .positionChange = 0
};

int16_t position = (int16_t)TIM1->CNT;
int16_t positionChange = position - positionPrev;

Command command = {.speed1 = 0, .speed2 = 0, .speed3 = 0, .throwerSpeed = 0, .delimiter = 0}; // (4)
volatile uint8_t isCommandReceived = 0; // (5)
volatile uint8_t driverReset = 0;
uint16_t timer = 0;
uint16_t enable_pid = 0;
Feedback feedback = { // (1)
	      .speed1 = 0,
	      .speed2 = 0,
	      .speed3 = 0,
	      .delimiter = 0xAAAA
	  };;



void CDC_On_Receive(uint8_t* buffer, uint32_t* length) { // (6)
  if (*length == sizeof(Command)) { // (7)
    memcpy(&command, buffer, sizeof(Command)); // (8)

    if (command.delimiter == 0xAAAA) { // (9)
      isCommandReceived = 1;
    }
    if (command.delimiter == 0xBBBB) { // (9)
    	  		driverReset = 1;
    		}
  }


//??tuleb üle vaadada
  ///kuegimobeetd koodist

void wake_drivers_up() {
      HAL_GPIO_WritePin(GPIOB, MOT_SLEEP_Pin, GPIO_PIN_RESET);
      for(uint16_t i = 0; i < 350; i++) __asm("nop");
  	HAL_GPIO_WritePin(GPIOB, MOT_SLEEP_Pin, GPIO_PIN_SET);
  	for(uint16_t i = 0; i < 10; i++) __asm("nop");
}

//??????


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  HAL_GPIO_TogglePin(GPIOF, GREEN_DBG_LED_1_Pin);
  TIM1->CCR1 = calc_pwm(0);
  TIM2->CCR1 = calc_pwm(1);
  TIM2->CCR3 = calc_pwm(2);

  if(!motor_status[0].forward) {
	TIM1->CCR2 = 0;
  } else {
	TIM1->CCR2 = 65535;
	  }
  if(!motor_status[1].forward) {
	TIM2->CCR2 = 0;
  } else {
	TIM2->CCR2 = 65535;
  }
  if(!motor_status[2].forward) {
	TIM2->CCR4 = 0;
  } else {
	TIM2->CCR4 = 65535;
  }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_USB_Device_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  Feedback feedback = { // (1)
        .speed1 = 0,
        .speed2 = 0,
        .speed3 = 0,
        .delimiter = 0xAAAA
    };

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = 9500;
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (isCommandReceived) { // (2)
		  isCommandReceived = 0;
	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // (3)

	      feedback.speed1 = motor1Control.speed; // (4)
	      feedback.speed2 = motor2Control.speed;
	      feedback.speed3 = motor3Control.speed;

	      CDC_Transmit_FS(&feedback, sizeof(feedback)); // (5)
	      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED_B1_Pin|LED_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Debug_NRST_GPIO_Port, Debug_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Mikro_M1_en1_Pin|Mikro_M1_en2_Pin|Mikro_M2_en1_Pin|Mikro_Visk_Pin
                          |LED_R1_Pin|LED_R2_Pin|Mikro_Palli_H_Pin|Mikro_Palli_S_Pin
                          |Mikro_M3_en2_Pin|Mikro_M3_en1_Pin|Mikro_M2_en2_Pin|Mikro_m3_en1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Mikro_M_sleep_Pin|Mikro_M_off_Pin|Mikro_m2_en1_Pin|Mikro_m2_en2_Pin
                          |Mikro_m1_en1_Pin|Mikro_m1_en2_Pin|Mikro_m3_en2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_B1_Pin LED_B2_Pin */
  GPIO_InitStruct.Pin = LED_B1_Pin|LED_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Debug_NRST_Pin */
  GPIO_InitStruct.Pin = Debug_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Debug_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mikro_M1_en1_Pin Mikro_M1_en2_Pin Mikro_M2_en1_Pin Mikro_Visk_Pin
                           LED_R1_Pin LED_R2_Pin Mikro_Palli_H_Pin Mikro_Palli_S_Pin
                           Mikro_M3_en2_Pin Mikro_M3_en1_Pin Mikro_M2_en2_Pin Mikro_m3_en1_Pin */
  GPIO_InitStruct.Pin = Mikro_M1_en1_Pin|Mikro_M1_en2_Pin|Mikro_M2_en1_Pin|Mikro_Visk_Pin
                          |LED_R1_Pin|LED_R2_Pin|Mikro_Palli_H_Pin|Mikro_Palli_S_Pin
                          |Mikro_M3_en2_Pin|Mikro_M3_en1_Pin|Mikro_M2_en2_Pin|Mikro_m3_en1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Mikro_M_sleep_Pin Mikro_M_off_Pin Mikro_m2_en1_Pin Mikro_m2_en2_Pin
                           Mikro_m1_en1_Pin Mikro_m1_en2_Pin Mikro_m3_en2_Pin */
  GPIO_InitStruct.Pin = Mikro_M_sleep_Pin|Mikro_M_off_Pin|Mikro_m2_en1_Pin|Mikro_m2_en2_Pin
                          |Mikro_m1_en1_Pin|Mikro_m1_en2_Pin|Mikro_m3_en2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
