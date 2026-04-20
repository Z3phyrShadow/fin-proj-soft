/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ORCAS v2 Main program body – STM32F030R8 port
  *
  *  Changes from STM32F103R8 version:
  *  - SystemClock_Config: removed RCC_CLOCKTYPE_PCLK2 (F030 has a single APB).
  *  - MX_I2C1_Init: replaced ClockSpeed + DutyCycle with F0 Timing register
  *    (0x00201D2B = 100 kHz standard mode at 8 MHz HSI).
  *  - MX_GPIO_Init: EXTI IRQn changed to EXTI4_15_IRQn (PC6 is in lines 4-15).
  *  - All stm32f1xx references removed; file now includes stm32f0xx headers via
  *    main.h.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_cmds_handle.h"
#include "orientation_ctrl_utils.h"
#ifdef AIMING_TOF_CTRL
#include "aiming_tof_ctrl.h"
#endif
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL   0x02
#define PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING 0x04
#define PERODIC_ROUTINES_DISPATCHED__AIMING_TOF_CTRL    0x08
uint8_t perodic_routines_dispatched = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Start master timer and UART receive */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

  /* Start PWM for Searchlight / Targeting LED */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* Initialize orientation control (steppers) */
  orientation_ctrl_init();

  /* Infinite loop */
  while (1)
  {
    if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL){
        perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL;
        orientation_ctrl_perodic_routines();
    }
    if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING){
        perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING;
        uart_cmds_handle_perodic_routines();
    }
  }
}

/**
  * @brief System Clock Configuration
  *
  *  STM32F030R8 running at 8 MHz HSI (no PLL).
  *  The F030 has a single APB bus (PCLK1 only) — RCC_CLOCKTYPE_PCLK2 does
  *  not exist and must NOT be passed to HAL_RCC_ClockConfig().
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* NOTE: STM32F030 has only one APB bus (PCLK1).
   *       RCC_CLOCKTYPE_PCLK2 does not exist on F0 and must be omitted. */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                     RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization – master 250 µs tick timer.
  *
  *  STM32F030R8: PCLK1 = 8 MHz (no PLL, AHB/APB1 divider = 1).
  *  TIM2 clock = 8 MHz.
  *  Prescaler = 7  →  f_count = 8 MHz / (7+1) = 1 MHz  →  T_count = 1 µs
  *  Period    = 250 →  interrupt every (250+1) µs ≈ 250 µs
  *
  *  NOTE: TIM2 on F030R8 is a 32-bit timer. The ARR / CNT registers are 32-bit
  *        but using 16-bit values here is fine.
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 7;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 250;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization – PWM for searchlight / targeting LED on PB1.
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 7;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 100;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  /* Searchlight CH4 → PB1 (AF1 configured in HAL_TIM_MspPostInit) */
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization – 9600 baud, 8N1.
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 9600;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization – standard mode 100 kHz for GY-TOF10M sensor.
  *
  *  STM32F030R8 I2C API change from F1:
  *    F1: hi2c1.Init.ClockSpeed = 100000; hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  *    F0: hi2c1.Init.Timing     = 0x00201D2B;  (pre-computed for 8 MHz, 100 kHz)
  *
  *  Timing = 0x00201D2B decoded:
  *    PRESC   = 0  → tPRESC = 1/8 MHz = 125 ns
  *    SCLDEL  = 2  → tSCLDEL = (2+1)*125 ns = 375 ns
  *    SDADEL  = 0
  *    SCLH    = 0x1D = 29 → tSCLH = (29+1)*125 ns = 3750 ns
  *    SCLL    = 0x2B = 43 → tSCLL = (43+1)*125 ns = 5500 ns
  *    Period ≈ 3750+5500 = 9250 ns → ~108 kHz (within ±10% of 100 kHz spec)
  *
  *  F0 I2C also adds OwnAddress2Masks field (not present on F1).
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.Timing          = 0x00201D2B;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization.
  *
  *  NOTE: EXTI IRQn changed from EXTI9_5_IRQn (F1) to EXTI4_15_IRQn (F030).
  *        On STM32F0 all EXTI lines 4-15 share one interrupt vector.
  *        The pan-origin limit switch is on PC6 (GPIO_PIN_6) which falls in
  *        this range.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TILT_MOTOR_STEP_Pin|TILT_MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAN_MOTOR_DIR_GPIO_Port, PAN_MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_D2_Pin|LED_D1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PAN_MOTOR_STEP_Pin (PC12) */
  GPIO_InitStruct.Pin   = PAN_MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TILT_MOTOR_STEP_Pin TILT_MOTOR_DIR_Pin LED_D2_Pin LED_D1_Pin */
  GPIO_InitStruct.Pin   = TILT_MOTOR_STEP_Pin|TILT_MOTOR_DIR_Pin|LED_D2_Pin|LED_D1_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_MOTOR_ORIGIN_Pin (PC6) – falling-edge EXTI */
  GPIO_InitStruct.Pin   = PAN_MOTOR_ORIGIN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_MOTOR_DIR_Pin (PD2) */
  GPIO_InitStruct.Pin   = PAN_MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAN_MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init – PC6 (GPIO_PIN_6) is in the EXTI4_15 group on F030  */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */
#define GLOBAL_TIMER_INT_PERIOD_US	250
uint16_t global_timer_count_us = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){
		global_timer_count_us += GLOBAL_TIMER_INT_PERIOD_US;
		if(global_timer_count_us >= 10000){
			global_timer_count_us = 0;
		}

		if(global_timer_count_us % UART_CMDS_HANDLE_ROUTINES_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING;
		}

		if(global_timer_count_us % ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
#ifdef PAN_CTRL
	if(GPIO_Pin == PAN_MOTOR_ORIGIN_Pin){
		orientation_ctrl_isr(GPIO_Pin);
	}
#endif
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
