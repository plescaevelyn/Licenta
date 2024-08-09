/*
 * rbwsLidarA1M8_inc.h
 *
 *  Created on: Apr 15, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSLIDARA1M8_INC_H_
#define INC_RBWSLIDARA1M8_INC_H_

/************/
/* Includes */
/************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*********************/
/* Macro definitions */
/*********************/
#define LIDAR_START_COMMAND_PACHET 0xA5	/* to be sent at the beginning of each command */
#define LIDAR_STOP 0x25					/* set Lidar operating mode to idle */
#define LIDAR_RESET 0x40
#define LIDAR_SCAN 0x20					/* enter scanning mode */
#define LIDAR_EXPRESS_SCAN 0x82			/* enter scanning state at highest speed */
#define LIDAR_FORCE_SCAN 0x21			/* enter scan state and force data out without checking rotation speed */
#define LIDAR_GET_INFO 0x50				/* get device info */
#define LIDAR_GET_HEALTH 0x52			/* get device health info */
#define LIDAR_GET_SAMPLERATE 0x59		/* get sampling time */
#define LIDAR_GET_CONFIG 0x84			/* get Lidar configuration */

/***********************/
/* Function prototypes */
/***********************/
void delay(int16_t ms);
void rbwsLidarA1M8_init();
void rbwsLidarA1M8_main();
uint8_t rbwsLidarA1M8_sendCommandAndGetResponse(uint8_t command, uint8_t* responsePacket);
uint8_t rbwsLidarScanMode();

#endif /* INC_RBWSLIDARA1M8_INC_H_ */
