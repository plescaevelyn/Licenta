/*
 * rbwsLidarA1M8_inc.h
 *
 *  Created on: Apr 15, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSLIDARA1M8_INC_H_
#define INC_RBWSLIDARA1M8_INC_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/****************************/
/* PROTOTYPES               */
/****************************/
void delay(int16_t ms);

void rbwsLidarA1M8_init();

void rbwsLidarA1M8_main();

uint8_t rbwsLidarA1M8_sendCommandAndGetResponse(uint8_t command, uint8_t* responsePacket);

uint8_t rbwsLidarScanMode();

#endif /* INC_RBWSLIDARA1M8_INC_H_ */
