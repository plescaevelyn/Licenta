/*
 * rbwsLidarA1M8_main.c
 *
 *  Created on: Apr 15, 2024
 *      Author: PLE1CLJ
 */

/************/
/* Includes */
/************/
#include <rbwsLidarA1M8_inc.h>
#include "rbwsMotorControl.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "task.h"

/************************/
/* Variable definitions */
/************************/
uint8_t scanStarted;

/****************************/
/* Function implementations */
/****************************/
/**
 * @brief Initialise Lidar measurement. Returns Health status of Lidar.
 */
void rbwsLidarA1M8_init()
{
	scanStarted = 0;
	uint8_t receiveBuffer[8];
	uint8_t responseLen;

	/* Reset RPLidar */
	responseLen = rbwsLidarA1M8_sendCommandAndGetResponse(LIDAR_RESET, receiveBuffer);

	/* Get device Health info */
	responseLen = rbwsLidarA1M8_sendCommandAndGetResponse(LIDAR_GET_HEALTH, receiveBuffer);
	PRINTF("Device health response: ");
	for(int i = 0; i < responseLen; i++)
	{
		PRINTF("%02X", receiveBuffer[i]);
	}
	PRINTF("\n");

	rbwsLidarScanMode();
}

/**
 * @brief Main function of Lidar.
 */

uint32_t statusFlags;

void rbwsLidarA1M8_main()
{
	rbwsLidarScanMode();
	uint8_t lidarScanData[5];

	LPUART_ReadBlocking(LPUART2, lidarScanData, 5);
    PRINTF("\nLidar: ");

    for(int i = 0; i < 5; i++)
    {
    	PRINTF("%02X ", lidarScanData[i]);
    }
    PRINTF("\n");

}

/**
 * @brief Delay function in milliseconds.
 */
void delay(int16_t ms)
{
	SDK_DelayAtLeastUs(ms * 1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}

/**
 * @brief Sends a command that selects the usecase of the Lidar and reads a response packet.
 * Returns the length of the read data.
 *
 * @param command is the functioning mode of the Lidar
 * @param responsePacket is the data received from the Lidar
 *
 */
uint8_t rbwsLidarA1M8_sendCommandAndGetResponse(uint8_t command, uint8_t* responsePacket)
{
	uint8_t hasDescriptor;
	uint8_t dataOut[2];
	uint8_t responseDescriptor[7];
	uint8_t dataInLen;

	dataOut[0] = LIDAR_START_COMMAND_PACHET;

	switch(command)
	{
		case LIDAR_RESET:
			dataInLen = 8;
			hasDescriptor = 0;
			dataOut[1] = LIDAR_RESET;
		break;
		case LIDAR_GET_HEALTH:
			dataInLen = 3;
			hasDescriptor = 1;
			dataOut[1] = LIDAR_GET_HEALTH;
		break;
		case LIDAR_SCAN:
			dataInLen = 5;
			hasDescriptor = 1;
			dataOut[1] = LIDAR_SCAN;
		break;
		case LIDAR_STOP:
			dataInLen = 0;
			hasDescriptor = 0;
		break;
		default:
			dataInLen = -1;
			return dataInLen;
		break;
	}

	LPUART_WriteBlocking(LPUART2, dataOut, 2);
	if(hasDescriptor == 1)
	{

		LPUART_ReadBlocking(LPUART2, responseDescriptor, 7);
		for(int i = 0; i < 7; i++)
		{
			PRINTF("%02X ", responseDescriptor[i]);
		}
		PRINTF("\n");
	}
	if(dataInLen > 0)
	{
		LPUART_ReadBlocking(LPUART2, responsePacket, dataInLen);
	}


	return dataInLen;
}

/**
 * @param Scan mode of the Lidar sensor
 */
uint8_t rbwsLidarScanMode()
{
	uint8_t responsePacket[8];
	uint8_t responseLength;

	/* motor power up */
	rbwsMotorControl_Set_LiDAR_Motor_DutyCycle(100);
	rbwsMotorControl_Generate_PWM(0);

	/* start scan */
	responseLength = rbwsLidarA1M8_sendCommandAndGetResponse(LIDAR_SCAN, responsePacket);

	return 0;
}
