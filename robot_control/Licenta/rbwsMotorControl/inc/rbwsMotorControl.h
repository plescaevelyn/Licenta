/*
 * rbwsMotorControl.h
 *
 *  Created on: Feb 19, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSMOTORCONTROL_H_
#define INC_RBWSMOTORCONTROL_H_

/************/
/* Includes */
/************/
#include <stdint.h>
#include <stdio.h>
#include <fsl_common.h>
#include <fsl_ftm.h>

/*********************/
/* Macro definitions */
/*********************/
/* The Flextimer base address/channel used for board */
#define FTM_BASEADDR		FTM0

#define LIDAR_FTM_CHANNEL  	kFTM_Chnl_1
#define MOTOR_FTM_CHANNEL	kFTM_Chnl_0
#define SERVO_FTM_CHANNEL	kFTM_Chnl_1
#define TIMER_PERIOD_US 	1000U
#define PRECISION			100

#define LIDAR_FTM_ADDRESS	FTM1
#define MOTOR_FTM_ADDRESS	FTM0

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_CoreSysClk)
#ifndef FTM_PWM_ON_LEVEL
#define FTM_PWM_ON_LEVEL kFTM_HighTrue
#endif
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY (5000U)
#endif
#ifndef FTM_CHANNEL_INTERRUPT_ENABLE
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl0InterruptEnable
#endif
#ifndef FTM_INTERRUPT_NUMBER
#define FTM_INTERRUPT_NUMBER 0
#endif

/*******************
 * Prototypes
 *******************/
void rbwsMotorControl_init();
void rbwsMotorControl_Generate_PWM(uint8_t usecase);
void rbwsMotorControl_Set_LiDAR_Motor_DutyCycle(uint16_t desired_value);
void rbwsMotorControl_Set_Motor_DutyCycle(uint16_t desired_value);
void rbwsMotorControl_Set_Servo_DutyCycle(uint16_t desired_value);

#endif /* INC_RBWSMOTORCONTROL_H_ */
