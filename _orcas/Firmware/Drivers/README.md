# Drivers – STM32F030R8 Port

The firmware has been ported from **STM32F103R8Tx** (F1 family) to **STM32F030R8Tx** (F0 family).
The HAL driver and CMSIS device headers must be updated accordingly.

## What you need to do

### 1. Replace the CMSIS Device headers

Delete `Drivers/CMSIS/Device/ST/STM32F1xx/` and replace it with the F0 equivalent from the
**STM32CubeF0** firmware package:

```
Drivers/CMSIS/Device/ST/STM32F0xx/Include/
    stm32f0xx.h
    stm32f030x8.h
    system_stm32f0xx.h
```

The `system_stm32f0xx.c` implementation is already provided at:
`Core/Src/system_stm32f0xx.c`

### 2. Replace the HAL Driver sources

Delete `Drivers/STM32F1xx_HAL_Driver/` and replace it with:

```
Drivers/STM32F0xx_HAL_Driver/
    Inc/   (stm32f0xx_hal.h, stm32f0xx_hal_gpio.h, stm32f0xx_hal_i2c.h, ...)
    Src/   (stm32f0xx_hal.c, stm32f0xx_hal_gpio.c, stm32f0xx_hal_i2c.c, ...)
```

These files come from the **STM32CubeF0** package. Download it from:
https://github.com/STMicroelectronics/STM32CubeF0

Or regenerate the project in **STM32CubeMX** with board = **NUCLEO-F030R8** and
copy the generated `Drivers/` directory here.

### 3. Update IDE project settings (STM32CubeIDE / Makefile)

Update the following in your build system:

| Setting | Old (F1) | New (F0) |
|---|---|---|
| Device | `STM32F103R8Tx` | `STM32F030R8Tx` |
| C preprocessor define | `STM32F103xB` | `STM32F030x8` |
| Include path | `Drivers/CMSIS/Device/ST/STM32F1xx/Include` | `Drivers/CMSIS/Device/ST/STM32F0xx/Include` |
| Include path | `Drivers/STM32F1xx_HAL_Driver/Inc` | `Drivers/STM32F0xx_HAL_Driver/Inc` |
| Source path | `Drivers/STM32F1xx_HAL_Driver/Src` | `Drivers/STM32F0xx_HAL_Driver/Src` |
| Linker script | `STM32F103R8TX_FLASH.ld` | `STM32F030R8TX_FLASH.ld` |
| Startup file | `Core/Startup/startup_stm32f103r8tx.s` | `Core/Startup/startup_stm32f030x8.s` |
| FPU | None (M3 softfp) | None (M0 softfp) |
| CPU | `-mcpu=cortex-m3` | `-mcpu=cortex-m0` |

> **TIP**: The easiest way to regenerate a correct `.cproject` / `.mxproject` is to:
> 1. Open STM32CubeMX, select NUCLEO-F030R8.
> 2. Configure the same pinout (see `Core/Inc/main.h` for pin definitions).
> 3. Generate for STM32CubeIDE.
> 4. Copy the generated `Drivers/` directory and `.cproject`/`.mxproject` into this folder.
> 5. Replace the CubeMX-generated `Core/Src/` and `Core/Inc/` files with the ones in this repo.

## Files already converted (no further action needed)

| File | Description |
|---|---|
| `Core/Inc/main.h` | Includes `stm32f0xx_hal.h`, updated pin comment, `EXTI4_15_IRQn` |
| `Core/Inc/stm32f0xx_hal_conf.h` | F0 module selection |
| `Core/Inc/stm32f0xx_it.h` | F0 interrupt prototypes (no M3 fault handlers) |
| `Core/Src/main.c` | `SystemClock_Config` (no PCLK2), `MX_I2C1_Init` (Timing register), `EXTI4_15_IRQn` |
| `Core/Src/stm32f0xx_hal_msp.c` | No AFIO, explicit `GPIO_AFx_xxx` alternate functions |
| `Core/Src/stm32f0xx_it.c` | No M3 fault handlers, `EXTI4_15_IRQHandler` for PC6 |
| `Core/Src/system_stm32f0xx.c` | F0 system clock update (no F105/F107 PLL2 paths) |
| `Core/Src/uart_cmds_handle.c` | `SR/DR` → `ISR/RDR` for ORE clear |
| `Core/Startup/startup_stm32f030x8.s` | Cortex-M0, F030 vector table |
| `STM32F030R8TX_FLASH.ld` | RAM = 8 KB (was 20 KB on F103) |
