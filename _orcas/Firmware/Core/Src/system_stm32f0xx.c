/**
  ******************************************************************************
  * @file    system_stm32f0xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M0 Device Peripheral Access Layer System Source File
  *          for STM32F030R8.
  *
  *  This file provides two functions and one global variable to use from
  *  user application:
  *    - SystemInit()           : Called at startup (before main) to configure
  *                               the vector table location. All clock config
  *                               is done by HAL_RCC_* in SystemClock_Config().
  *    - SystemCoreClock        : Contains the core clock (HCLK) frequency.
  *    - SystemCoreClockUpdate(): Updates SystemCoreClock from live RCC registers.
  *
  *  Default after reset: HSI 8 MHz, no PLL, SystemCoreClock = 8 000 000.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "stm32f0xx.h"

/* ─── Private defines ──────────────────────────────────────────────────────── */
#if !defined (HSE_VALUE)
  #define HSE_VALUE  8000000U   /*!< Default External oscillator value in Hz */
#endif

#if !defined (HSI_VALUE)
  #define HSI_VALUE  8000000U   /*!< Default Internal oscillator value in Hz */
#endif

/* #define USER_VECT_TAB_ADDRESS */   /* Uncomment to enable VTOR relocation */

#if defined(USER_VECT_TAB_ADDRESS)
/* #define VECT_TAB_SRAM */           /* Uncomment to relocate to SRAM */
#if defined(VECT_TAB_SRAM)
  #define VECT_TAB_BASE_ADDRESS  SRAM_BASE
#else
  #define VECT_TAB_BASE_ADDRESS  FLASH_BASE
#endif
  #define VECT_TAB_OFFSET        0x00000000U
#endif /* USER_VECT_TAB_ADDRESS */

/* ─── Private variables ─────────────────────────────────────────────────────
   SystemCoreClock is updated by SystemCoreClockUpdate() or by the HAL when
   HAL_RCC_ClockConfig() is called. After reset the device runs at HSI 8 MHz. */
uint32_t SystemCoreClock = 8000000U;

/* AHB prescaler encoding table: index = HPRE bits [7:4] of RCC_CFGR */
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
/* APB prescaler encoding table: index = PPRE bits [10:8] of RCC_CFGR */
const uint8_t APBPrescTable[8U]  = {0, 0, 0, 0, 1, 2, 3, 4};

/* ─── SystemInit ──────────────────────────────────────────────────────────── */
/**
  * @brief  Setup the microcontroller system.
  *         On F0 this only optionally relocates the vector table.
  *         All clock setup is performed later in SystemClock_Config() via HAL.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
#endif
}

/* ─── SystemCoreClockUpdate ──────────────────────────────────────────────── */
/**
  * @brief  Update SystemCoreClock variable according to the current RCC
  *         register values. Must be called whenever the core clock changes.
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp;
  uint32_t pllmull;
  uint32_t pllsource;

  /* Determine active SYSCLK source ----------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case RCC_CFGR_SWS_HSI:   /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;

    case RCC_CFGR_SWS_HSE:   /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;

    case RCC_CFGR_SWS_PLL:   /* PLL used as system clock */
      /* Get PLL multiplication factor */
      pllmull  = (RCC->CFGR & RCC_CFGR_PLLMUL) >> 18U;
      pllmull  = pllmull + 2U;   /* PLLMUL bits encode (multiplier - 2) */
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

      if (pllsource == RCC_CFGR_PLLSRC_HSI_DIV2)
      {
        /* HSI/2 selected as PLL input */
        SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
      }
      else
      {
        /* HSE selected as PLL input (no pre-divider on F030) */
        SystemCoreClock = HSE_VALUE * pllmull;
      }
      break;

    default: /* HSI fallback */
      SystemCoreClock = HSI_VALUE;
      break;
  }

  /* Apply AHB prescaler */
  tmp = AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> 4U];
  SystemCoreClock >>= tmp;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
