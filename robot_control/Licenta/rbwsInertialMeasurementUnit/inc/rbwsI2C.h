/*
 * rbwsI2C.h
 *
 *  Created on: Jan 30, 2023
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSI2C_H_
#define INC_RBWSI2C_H_

#include "fsl_common.h"
#include "fsl_lpi2c.h"

/*********************/
/* Macro definitions */
/*********************/
#define BOARD_ACCEL_I2C_BASEADDR 		LPI2C0
#define LPI2C_CLOCK_FREQUENCY    		CLOCK_GetIpFreq(kCLOCK_Lpi2c0)
#define I2C_BAUDRATE             		100000

#define LPI2C_MASTER_BASE      		 	(LPI2C0_BASE)
#define LPI2C_MASTER_CLOCK_FREQUENCY 	kCLOCK_ScgSysOscClk
#define WAIT_TIME                   	100U
#define LPI2C_MASTER 				 	((LPI2C_Type *)LPI2C_MASTER_BASE)
#define LPI2C_DATA_LENGTH            	2U
#define LPI2C_SLAVE_ADDR_7BIT 			0x68

/* Status types */
#define I2C_OK					 		1
#define I2C_ERROR				 		0

/********************/
/* Type definitions */
/********************/
typedef enum _read_seq_command
{
    READ_STATUS     = 0U,
    READ_ACCEL_DATA = 1U,
} read_seq_command;

/*!
 * @brief This structure defines the Write command List.
 */
typedef struct _regList
{
    uint8_t reg; /* Register Address where the value is wrote to */
    uint8_t val;
    uint8_t size; /* read size from register */
} regList_t;

/*******************
 * Prototypes
 *******************/
void lpi2c_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData);
status_t rbwsI2C_Read(uint8_t* rxBuff,uint8_t* txBuff, uint8_t rxlength, uint8_t txlength);
status_t rbwsI2C_Write(uint8_t* txbuff, uint8_t length);

#endif /* INC_RBWSI2C_H */
