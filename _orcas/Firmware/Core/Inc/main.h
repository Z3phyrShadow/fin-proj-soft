/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   ORCAS v2 – Laser turret build.
  *
  *                   ROLE: MOTOR CONTROL ONLY.
  *                   All sensors (camera, TOF, ultrasonic) live on the
  *                   Raspberry Pi. The Pi runs YOLO, computes angle errors,
  *                   and sends UART commands. The STM32 only drives the two
  *                   stepper motors (pan/tilt) and the searchlight LED.
  *                   Connection to Pi: USB-serial (Nucleo onboard ST-Link)
  *                   at 9600 baud.
  *
  *                   Hardware: STM32F030R8 Nucleo + CNC Shield V3 + 2x A4988
  *                   Pan motor  : X-axis A4988  (PC12 STEP / PD2 DIR)
  *                   Tilt motor : Y-axis A4988  (PB3  STEP / PB4 DIR)
  *                   Pan origin : PC6  (active-LOW limit switch, EXTI falling)
  *                   UART       : USART1 PA9/PA10, 9600 baud (via USB-serial)
  *                   Searchlight: TIM3 CH4 PWM → PB1 (AF1)
  *
  *  Compile-time feature flags (sensors disabled – all handled by Pi):
  *    #define PAN_CTRL   – enable pan stepper
  *    #define TILT_CTRL  – enable tilt stepper (step-count mode)
  *    FIRE_CTRL        DISABLED – laser fired via Pi GPIO + MOSFET
  *    TOF_SENSORS_CTRL DISABLED – TOF/ultrasonic read by Pi, not STM32
  *    AIMING_TOF_CTRL  DISABLED – TOF on Pi I2C, not STM32 I2C
  *    TILT_USE_ACCEL   DISABLED – no accelerometer on STM32
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
#include "stm32f0xx_hal.h"

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
/* Motor control (on STM32) */
#define PAN_CTRL   /* Pan stepper enabled                                      */
#define TILT_CTRL  /* Tilt stepper enabled (step-count mode, no accelerometer) */

/* Disabled: sensors and firing are handled by the Raspberry Pi, not STM32   */
/* #define TILT_USE_ACCEL   */ /* No accelerometer on STM32 in this build      */
/* #define FIRE_CTRL        */ /* Laser fired via Pi GPIO + MOSFET             */
/* #define TOF_SENSORS_CTRL */ /* TOF/ultrasonic read by Pi USB sensors        */
/* #define AIMING_TOF_CTRL  */ /* TOF sensor is on Pi I2C, not STM32 I2C       */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
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
/* NOTE: On F030 EXTI lines 4-15 share one IRQ: EXTI4_15_IRQn               */
#define PAN_MOTOR_ORIGIN_EXTI_IRQn  EXTI4_15_IRQn

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

/* ── Searchlight / targeting LED (TIM3 CH4 PWM → PB1, AF1) ─────────────  */
#define TARGETING_LED_PWM_Pin       GPIO_PIN_1
#define TARGETING_LED_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* NOTE: No I2C peripherals are used on the STM32 in this build.
 *       The TOF sensor and all other sensors are connected to and read by
 *       the Raspberry Pi. PB6/PB7/PB10/PB11 are unused GPIO in this config. */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
