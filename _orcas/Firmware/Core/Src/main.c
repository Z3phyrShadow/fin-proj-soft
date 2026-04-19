/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ORCAS v2 Main program body
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
I2C_HandleTypeDef hi2c1;

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
static void MX_I2C1_Init(void);
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
#ifdef AIMING_TOF_CTRL
  MX_I2C1_Init();
#endif
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Start master timer and UART receive */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

  /* Start PWM for Searchlight / Targeting LED */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* Initialize orientation control (steppers) */
  orientation_ctrl_init();

#ifdef AIMING_TOF_CTRL
  aiming_tof_ctrl_init();
#endif

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
#ifdef AIMING_TOF_CTRL
    if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__AIMING_TOF_CTRL){
        perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__AIMING_TOF_CTRL;
        aiming_tof_ctrl_perodic_routines();
    }
#endif
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  /* Searchlight CH4 */
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

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

  /*Configure GPIO pin : PAN_MOTOR_STEP_Pin */
  GPIO_InitStruct.Pin = PAN_MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TILT_MOTOR_STEP_Pin TILT_MOTOR_DIR_Pin LED_D2_Pin LED_D1_Pin */
  GPIO_InitStruct.Pin = TILT_MOTOR_STEP_Pin|TILT_MOTOR_DIR_Pin|LED_D2_Pin|LED_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_MOTOR_ORIGIN_Pin */
  GPIO_InitStruct.Pin = PAN_MOTOR_ORIGIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = PAN_MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAN_MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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

#ifdef AIMING_TOF_CTRL
		if(global_timer_count_us % AIMING_TOF_CTRL_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__AIMING_TOF_CTRL;
		}
#endif
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

void MX_I2C_ForceClearBusyFlag(I2C_HandleTypeDef *hi2c,
                               GPIO_TypeDef* sda_gpio_port, uint16_t sda_gpio_pin,
                               GPIO_TypeDef* scl_gpio_port, uint16_t scl_gpio_pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = sda_gpio_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(sda_gpio_port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = scl_gpio_pin;
	HAL_GPIO_Init(scl_gpio_port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(sda_gpio_port, sda_gpio_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_SET);

	for(uint8_t i = 0; i < 9; i++){
		HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}

	HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sda_gpio_port, sda_gpio_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sda_gpio_port, sda_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_I2C_Init(hi2c);
}
