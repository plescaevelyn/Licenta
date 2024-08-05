/*
 * rbwsFreeRTOS.h
 *
 *  Created on: Feb 7, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSFREERTOS_H_
#define INC_RBWSFREERTOS_H_

#include <stdint.h>
#include <stdio.h>

/* Task priorities. */
#define GET_INIT_TASK_PRIORITY ( configMAX_PRIORITIES - 1 )
#define GET_MAIN_TASK_PRIORITY ( configMAX_PRIORITIES - 2 )
#define DONE_CREATE_INIT		1U
#define DONE_CREATE_MAIN		2U
#define DONE_INIT				3U
#define DONE_MAIN				4U
#define FAILED					100U

void rbwsFreeRTOS_Initialize(void);
void rbwsFreeRTOS_Initialize_Task(void *pvParams);
void rbwsFreeRTOS_Main_Task(void *pvParams);
void rbwsFreeRTOS_Task_Printer(void *arg);

#endif /* INC_RBWSFREERTOS_H_ */
