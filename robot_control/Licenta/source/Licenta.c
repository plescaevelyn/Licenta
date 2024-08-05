/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Licenta.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "FreeRTOS.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE16Z4.h"
#include "fsl_debug_console.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "clock_config.h"

#include "rbwsI2C.h"
#include "rbwsKalmanFilter.h"
#include "rbwsLidarA1M8_inc.h"
#include "rbwsLidarA1M8_api.h"
#include "rbwsMPU6050.h"
#include "rbwsMotorControl.h"
#include "rbwsPWT.h"

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */

	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    /* Initialize functions */
	rbwsLidarA1M8_init();
	rbwsMPU6050_init();
    rbwsMotorControl_init();

	/* Main functions */
	while(1)
	{
		rbwsLidarA1M8_main();
		rbwsMPU6050_main();
		rbwsPWT_main();
	}

//    rbwsFreeRTOS_Initialize();

    return 0 ;
}
