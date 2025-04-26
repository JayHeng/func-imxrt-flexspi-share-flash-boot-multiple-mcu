/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_BOOT_START (0x00002000)
#define APP_BOOT_ADDRESS (void *)APP_BOOT_START

extern unsigned char app_image_start[];
#define APP_IMAGE_START app_image_start

uint32_t s_appStack = 0;
uint32_t s_appEntry = 0;
 
typedef void (*call_function_t)(void);
call_function_t s_callFunction = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t get_app_image_size(void)
{
    uint32_t image_size;

#pragma section = "__app_image"
    image_size = (uint32_t)__section_end("__app_image") - (uint32_t)&app_image_start;

    return image_size;
}

void jump_to_app(void)
{
    s_appStack = *(uint32_t *)(APP_BOOT_START);
    s_appEntry = *(uint32_t *)(APP_BOOT_START + 4);
 
    // Turn off interrupts.
    __disable_irq();
 
    // Set the VTOR to default.
    SCB->VTOR = APP_BOOT_START;
 
    // Memory barriers for good measure.
    __ISB();
    __DSB();
 
    // Set main stack pointer and process stack pointer.
    __set_MSP(s_appStack);
    __set_PSP(s_appStack);
 
    // Jump to app entry point, does not return.
    s_callFunction = (call_function_t)s_appEntry;
    s_callFunction();
 
    while (1)
    {
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();

    PRINTF("\r\n---------------------------------------.\r\n");
    PRINTF("Hello boot_loader.\r\n");

    uint32_t app_image_size;
    app_image_size = get_app_image_size();
    (void)PRINTF("Copy boot_app image to address: 0x%x, size: %d\r\n", (void *)(char *)APP_BOOT_ADDRESS,
                 app_image_size);

    /* Copy application from FLASH to the target memory. */
    (void)memcpy((void *)(char *)APP_BOOT_ADDRESS, (void *)APP_IMAGE_START, app_image_size);

    PRINTF("Jump into boot_app.\r\n");
    jump_to_app();

    while (1)
    {
    }
}
