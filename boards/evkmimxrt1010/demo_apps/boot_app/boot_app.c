/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "board.h"
#include "app.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*
 * App mode definition.
 */
typedef enum _app_mode
{
    APP_ReleaseSlaveMcu = 0,
    APP_BlinkyLed       = 1,
    APP_AccessFlash     = 2,
    
    APP_ModeEnd = APP_AccessFlash
} app_mode_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

status_t bsp_rw_nor_flash(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static app_mode_t s_targetAppMode;

volatile uint32_t g_systickCounter;

/*******************************************************************************
 * Code
 ******************************************************************************/

void bsp_init_func_gpio(void) 
{
    gpio_pin_config_t GPIO_11_config = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode
    };
    GPIO_PinInit(GPIO1, 11U, &GPIO_11_config);
    IOMUXC_SetPinMux(IOMUXC_GPIO_11_GPIOMUX_IO11, 0U); 
    IOMUXC_GPR->GPR26 = ((IOMUXC_GPR->GPR26 &
      (~(BOARD_INITPINS_IOMUXC_GPR_GPR26_GPIO_SEL_MASK))) 
        | IOMUXC_GPR_GPR26_GPIO_SEL(0x00U)      
      );
    IOMUXC_SetPinConfig(IOMUXC_GPIO_11_GPIOMUX_IO11, 0x10A0U); 
}

void bsp_deinit_flexspi_pins(void) 
{
    // SW_MUX_CTL
    // bit4 - SION
    //     0 DISABLED — Input Path is determined by functionality
    // bit2-0 - MUX_MODE
    //     101 ALT5 — Select mux mode: ALT5 mux port: GPIO
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_06_GPIO2_IO06, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_07_GPIO2_IO07, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_09_GPIO2_IO09, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_10_GPIO2_IO10, 0U);
    // SW_PAD_CTL
    // bit16 - HYS
    //     0 HYS_0_Hysteresis_Disabled — Hysteresis Disabled
    // bit15-14 - PUS
    //     00 PUS_0_100K_Ohm_Pull_Down — 100K Ohm Pull Down
    // bit13 - PUE
    //     0 PUE_0_Keeper — Keeper
    // bit12 - PKE
    //     1 PKE_1_Pull_Keeper_Enabled — Pull/Keeper Enabled
    // bit11 - ODE
    //     0 ODE_0_Open_Drain_Disabled — Open Drain Disabled
    // bit10-8 reserved
    // bit7-6 - SPEED
    //     10 SPEED_2_fast_150MHz — fast(150MHz)
    // bit5-3 - DSE
    //     100 DSE_4_R0_4 — R0/4
    // bit2-1 reserved
    // bit0 - SRE
    //     0 SRE_0_Slow_Slew_Rate — Slow Slew Rate
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_06_GPIO2_IO06, 0x10A0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_07_GPIO2_IO07, 0x10A0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_09_GPIO2_IO09, 0x10A0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_10_GPIO2_IO10, 0x10A0U);
}

void bsp_hold_slave_mcu(void) 
{
    GPIO_PinWrite(GPIO1, 11, 0U);
}

void bsp_release_slave_mcu(void) 
{
    GPIO_PinWrite(GPIO1, 11, 1U);
}

void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
    else
    {
        g_systickCounter = 1000U;
        GPIO_PortToggle(GPIO1, 1 << 11);
    }
}

void bsp_init_SysTick(void)
{
    g_systickCounter = 1000U;
    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }
}

void bsp_init_flexspi_pins(void) 
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_06_FLEXSPI_A_SS0_B, 1U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_07_FLEXSPI_A_DATA1, 1U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_09_FLEXSPI_A_DATA0, 1U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_10_FLEXSPI_A_SCLK, 1U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_06_FLEXSPI_A_SS0_B, 0x10E1U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_07_FLEXSPI_A_DATA1, 0x10E1U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_09_FLEXSPI_A_DATA0, 0x10E1U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_10_FLEXSPI_A_SCLK, 0x10E1U); 
}

void APP_ModeSwitch(app_mode_t targetAppMode)
{
    switch (targetAppMode)
    {
        case APP_ReleaseSlaveMcu:
            PRINTF("- Release Slave MCU mode\r\n");
            bsp_deinit_flexspi_pins();
            bsp_release_slave_mcu();
            break;
        case APP_BlinkyLed:
            PRINTF("- Blinky LED mode\r\n");
            bsp_init_SysTick();
            break;
        case APP_AccessFlash:
            PRINTF("- Access NOR Flash mode\r\n");
            bsp_hold_slave_mcu();
            bsp_init_flexspi_pins();
            bsp_rw_nor_flash();
            break;
        default:
            assert(false);
            break;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    char ch;

    /* Init board hardware. */
    BOARD_InitHardware();
    bsp_init_func_gpio();

    PRINTF("\r\n---------------------------------------.\r\n");
    PRINTF("Hello boot_app.\r\n");

    while (1)
    {
        PRINTF("\r\nSelect the desired operation \r\n");
        PRINTF("Press  %c for enter: Release Slave MCU mode\r\n",
               (uint8_t)'A' + (uint8_t)APP_ReleaseSlaveMcu);
        PRINTF("Press  %c for enter: Blinky LED mode\r\n",
               (uint8_t)'A' + (uint8_t)APP_BlinkyLed);
        PRINTF("Press  %c for enter: Access NOR Flash mode\r\n",
               (uint8_t)'A' + (uint8_t)APP_AccessFlash);
        PRINTF("Waiting for app mode select...\r\n");

        /* Wait for user response */
        ch = GETCHAR();

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }

        s_targetAppMode = (app_mode_t)(ch - 'A');

        if (s_targetAppMode <= APP_ModeEnd)
        {
            APP_ModeSwitch(s_targetAppMode);
        }
        PRINTF("Next loop\r\n");
    }
}
