/*
 * rbwsFreeRTOS.c
 *
 *  Created on: Feb 7, 2024
 *      Author: PLE1CLJ
 */


/************/
/* Includes */
/************/
#include "rbwsFreeRTOS.h"
#include "FreeRTOS.h"

#include "rbwsI2C.h"
#include "rbwsLidarA1M8_inc.h"
#include "rbwsMPU6050.h"
#include "rbwsMotorControl.h"
#include "rbwsPWT.h"

#include "task.h"

/*********************/
/* Macro definitions */
/*********************/
#define INIT_TASK_PRIORITY				  ( configMAX_PRIORITIES - 1 )
#define MAIN_TASK_PRIORITY                ( configMAX_PRIORITIES - 2 )

/************************/
/* Variable definitions */
/************************/
volatile uint8_t check_for_debugger=0;

/****************************/
/* Function implementations */
/****************************/
/**
 * @brief Performs the initialization of modules.
 *
 * This function initializes various modules, such as the distance sensing module, motor control module, ECU Manager module,
 * to prepare them for operation.
 */

void rbwsFreeRTOS_Module_Init()
{
	rbwsLidarA1M8_init();

    rbwsMotorControl_Set_Motor_DutyCycle(50);
    rbwsMotorControl_Set_Servo_DutyCycle(30);

	rbwsMPU6050_init();
}

/**
 * @brief Performs the main function of modules.
 *
 * This function executes the main functionality of the modules during the runtime of the program.
 */

void rbwsFreeRTOS_Module_Main()
{
    rbwsMotorControl_Generate_PWM(1);
    rbwsMotorControl_Generate_PWM(2);

    rbwsMPU6050_main();

	rbwsLidarA1M8_main();

	rbwsPWT_main();
}

/**
 * @brief Initializes the FreeRTOS task for module initialization.
 *
 * This function is used to initialize a FreeRTOS task that handles module
 * initialization. It first checks for a debugger, performs module initialization,
 * and then suspends itself to let the Main_Task to initialize.
 */

void rbwsFreeRTOS_Initialize_Task(void *pvParams)
{
	(void) pvParams;

	taskENTER_CRITICAL();
	taskEXIT_CRITICAL();
	vTaskSuspend(NULL);
}

/**
 * @brief FreeRTOS task for executing the main function of modules.
 *
 * This FreeRTOS task executes the main functionality of various modules continuously,
 * at regular intervals. It increments a variable to check for a debugger,
 * performs the main module functions, and then delays for a specified period
 * before executing again (20ms).
 */

void rbwsFreeRTOS_Main_Task(void *pvParams)
{
	(void) pvParams;
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

/**
 * @brief Initializes the FreeRTOS tasks and starts the scheduler.
 *
 * This function sets up and initializes two FreeRTOS tasks, "InitTask" and "MainTask,"
 * and starts the FreeRTOS scheduler. If the tasks are successfully created, the scheduler
 * is started to begin the execution of these tasks.
 */

void rbwsFreeRTOS_Initialize(void)
{
	BaseType_t taskStatus;
	uint8_t ok=1;

	taskStatus = xTaskCreate( rbwsFreeRTOS_Initialize_Task, ( const char * const ) "InitTask", configMINIMAL_STACK_SIZE, (void*)0, INIT_TASK_PRIORITY + 1, NULL);
	if (taskStatus != pdPASS)
	{
		ok = 0;
	}

	taskStatus = xTaskCreate( rbwsFreeRTOS_Main_Task, ( const char * const ) "MainTask", configMINIMAL_STACK_SIZE, (void*)0, MAIN_TASK_PRIORITY, NULL);
	if (taskStatus != pdPASS)
	{
		ok = 0;
	}

	if (ok)
	{
		vTaskStartScheduler();
	}
}
