/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   ORCAS v2 – Laser turret build.
  *                   Hardware: STM32F103 Nucleo + CNC Shield V3 + 2x A4988
  *                   Pan motor  : X-axis A4988  (PC12 STEP / PD2 DIR)
  *                   Tilt motor : Y-axis A4988  (PB3  STEP / PB4 DIR)
  *                   Pan origin : PC6  (active-LOW limit switch, EXTI falling)
  *                   UART       : USART1 PA9/PA10, 9600 baud
  *                   Searchlight: TIM3 CH4 PWM (searchlight/targeting LED)
  *
  *  Compile-time feature flags:
  *    #define PAN_CTRL          – enable pan stepper
  *    #define TILT_CTRL         – enable tilt stepper (step-count mode, no
  *                               accelerometer; define TILT_USE_ACCEL if
  *                               the MMA845x is fitted)
  *    FIRE_CTRL   is DISABLED   – laser fired from Pi GPIO, not STM32
  *    TOF_SENSORS_CTRL DISABLED – no radar / ultrasonic sensor
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* ── Firmware version string (must be exactly 15 characters + NUL) ──────── */
#define FW_VER  "ORCAS_v2_202604"

/* ── Feature flags ──────────────────────────────────────────────────────── */
#define PAN_CTRL          /* Pan stepper enabled                              */
#define TILT_CTRL         /* Tilt stepper enabled (step-count mode)           */
/* #define TILT_USE_ACCEL */ /* Uncomment only if MMA845x is physically fitted */
/* #define FIRE_CTRL      */ /* DISABLED – laser fired by Pi GPIO, not STM32  */
/* #define TOF_SENSORS_CTRL */ /* DISABLED – no radar / ultrasonic             */
#define AIMING_TOF_CTRL   /* GY_TOF10M distance polling only                  */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MX_I2C_ForceClearBusyFlag(I2C_HandleTypeDef *hi2c,
                               GPIO_TypeDef* sda_gpio_port, uint16_t sda_gpio_pin,
                               GPIO_TypeDef* scl_gpio_port, uint16_t scl_gpio_pin);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* ── Pan motor (CNC Shield X-axis) ─────────────────────────────────────── */
#define PAN_MOTOR_STEP_Pin          GPIO_PIN_12
#define PAN_MOTOR_STEP_GPIO_Port    GPIOC
#define PAN_MOTOR_DIR_Pin           GPIO_PIN_2
#define PAN_MOTOR_DIR_GPIO_Port     GPIOD

/* ── Pan origin limit switch ─────────────────────────────────────────────  */
#define PAN_MOTOR_ORIGIN_Pin        GPIO_PIN_6
#define PAN_MOTOR_ORIGIN_GPIO_Port  GPIOC
#define PAN_MOTOR_ORIGIN_EXTI_IRQn  EXTI9_5_IRQn

/* ── Tilt motor (CNC Shield Y-axis) ─────────────────────────────────────  */
#define TILT_MOTOR_STEP_Pin         GPIO_PIN_3
#define TILT_MOTOR_STEP_GPIO_Port   GPIOB
#define TILT_MOTOR_DIR_Pin          GPIO_PIN_4
#define TILT_MOTOR_DIR_GPIO_Port    GPIOB

/* ── Status LEDs on Nucleo ───────────────────────────────────────────────  */
#define LED_D2_Pin                  GPIO_PIN_8
#define LED_D2_GPIO_Port            GPIOB
#define LED_D1_Pin                  GPIO_PIN_9
#define LED_D1_GPIO_Port            GPIOB

/* ── Searchlight / targeting LED (TIM3 CH4 PWM) ─────────────────────────  */
#define TARGETING_LED_PWM_Pin       GPIO_PIN_1
#define TARGETING_LED_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* I2C1 pins - used by GY_TOF10M sensor                                     */
#define I2C1_SDA_Pin        GPIO_PIN_7
#define I2C1_SDA_GPIO_Port  GPIOB
#define I2C1_SCL_Pin        GPIO_PIN_6
#define I2C1_SCL_GPIO_Port  GPIOB

/* I2C2 pins – retained in case TILT_USE_ACCEL is re-enabled later          */
#define I2C2_SDA_Pin        GPIO_PIN_11
#define I2C2_SDA_GPIO_Port  GPIOB
#define I2C2_SCL_Pin        GPIO_PIN_10
#define I2C2_SCL_GPIO_Port  GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
