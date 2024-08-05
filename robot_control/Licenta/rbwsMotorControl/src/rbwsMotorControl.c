/*
* @file rbwsMotorControl.c
*
* @brief This module contains the initialization of the FTM driver and the PWM module.
*
* ---------------------------------------------------------------------------
*
* List of authors (maybe incomplete due to the right to informational self-
*
* determination) It is not allowed to remove authors except the own name.
*
* @author: Plesca Evelyn-Iulia (PLE1CLJ)
*
* ------------------------------------------------------------------*/

#include "rbwsMotorControl.h"
#include <stdio.h>
#include <stdint.h>
#include "fsl_common.h"
#include "fsl_common_arm.h"
#include "fsl_ftm.h"
#include "fsl_debug_console.h"

/***************************/
/* 		Variables          */
/***************************/
volatile uint16_t motorUpdatedDutycycle = 0;
volatile uint16_t lidarUpdatedDutyCycle = 0;
volatile uint16_t servoUpdatedDutyCycle = 0;

/****************************/
/* Function implementations */
/****************************/
void delay(int16_t ms)
{
	SDK_DelayAtLeastUs(ms * 1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}

/**
 * @brief	PWM generation
 * @details usecase shall be 0 for the LiDAR, 1 for the motor and 2 for the servo motor
 *
 * @param	usecase
 */
void rbwsMotorControl_Generate_PWM(uint8_t usecase)
{
	switch (usecase)
	{
	case 0:
	    /* Update PWM duty cycle */
		FTM_UpdatePwmDutycycle(LIDAR_FTM_ADDRESS, LIDAR_FTM_CHANNEL, kFTM_EdgeAlignedPwm, lidarUpdatedDutyCycle);

		/* Set software trigger */
		FTM_SetSoftwareTrigger(LIDAR_FTM_ADDRESS, true);

        break;
	case 1:
	    /* Update PWM duty cycle */
		FTM_UpdatePwmDutycycle(MOTOR_FTM_ADDRESS, MOTOR_FTM_CHANNEL, kFTM_EdgeAlignedPwm, motorUpdatedDutycycle);

		/* Set software trigger */
		FTM_SetSoftwareTrigger(MOTOR_FTM_ADDRESS, true);

        break;
	case 2:
	    /* Update PWM duty cycle */
		FTM_UpdatePwmDutycycle(MOTOR_FTM_ADDRESS, SERVO_FTM_CHANNEL, kFTM_EdgeAlignedPwm, servoUpdatedDutyCycle);

		/* Set software trigger */
		FTM_SetSoftwareTrigger(MOTOR_FTM_ADDRESS, true);

        break;
    default:
    	PRINTF("Incorrect use case chosen!");
    	break;
	}
}

/**
 *
 * @brief       This function sets the duty cycle of the LiDAR's motor to its desired value.
 *
 * @param  		desired_value the desired value of the PWM
 */
void rbwsMotorControl_Set_LiDAR_Motor_DutyCycle(uint16_t desired_value)
{
	lidarUpdatedDutyCycle = desired_value;

    /* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(LIDAR_FTM_ADDRESS, LIDAR_FTM_CHANNEL, kFTM_EdgeAlignedPwm, lidarUpdatedDutyCycle);

	/* Set software trigger */
	FTM_SetSoftwareTrigger(LIDAR_FTM_ADDRESS, true);
}

/**
 *
 * @brief       This function sets the duty cycle of the motor to its desired value.
 *
 * @param  		desired_value the desired value of the PWM
 */
void rbwsMotorControl_Set_Motor_DutyCycle(uint16_t desired_value)
{
	motorUpdatedDutycycle = desired_value;

    /* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(MOTOR_FTM_ADDRESS, MOTOR_FTM_CHANNEL, kFTM_EdgeAlignedPwm, motorUpdatedDutycycle);

	/* Set software trigger */
	FTM_SetSoftwareTrigger(MOTOR_FTM_ADDRESS, true);
}

/**
 *
 * @brief       This function sets the duty cycle of the servo motor to its desired value.
 *
 * @param  		desired_value the desired value of the PWM
 */
void rbwsMotorControl_Set_Servo_DutyCycle(uint16_t desired_value)
{
	servoUpdatedDutyCycle = desired_value;

    /* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(MOTOR_FTM_ADDRESS, SERVO_FTM_CHANNEL, kFTM_EdgeAlignedPwm, servoUpdatedDutyCycle);

	/* Set software trigger */
	FTM_SetSoftwareTrigger(MOTOR_FTM_ADDRESS, true);
}

/**
 * @brief Initialize ESC
 *
 * @description Init ESC to 2ms and then 1.5ms so that it doesn't 'scream' anymore
 */
void rbwsMotorControl_initESC()
{
//	rbwsMotorControl_Set_Motor_DutyCycle(87); /* 8.75 - 2   ms */

	rbwsMotorControl_Set_Motor_DutyCycle(75);	/* 7.5 - 1.5 ms */

	delay(3000);
}

/**
 * @brief Initialize motor control function
 */
void rbwsMotorControl_init()
{
	rbwsMotorControl_Set_Motor_DutyCycle(0);
	rbwsMotorControl_Set_Servo_DutyCycle(0);

    rbwsMotorControl_Generate_PWM(1);
    rbwsMotorControl_Generate_PWM(2);

    rbwsMotorControl_initESC();
}
