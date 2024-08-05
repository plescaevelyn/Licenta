/*
* @file rbwsInertialMeasurementUnit.c
*
* @brief This module contains the initialization of the I2C protocol
* * ---------------------------------------------------------------------------
*
* List of authors (maybe incomplete due to the right to informational self-
*
* determination) It is not allowed to remove authors except the own name.
*
* @author: Plesca Evelyn-Iulia (PLE1CLJ)
*
* ------------------------------------------------------------------*/

#include "rbwsI2C.h"

#include <stdint.h>
#include <stdio.h>

#include "fsl_lpi2c.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool completionFlag = false;
volatile bool nakFlag        = false;

size_t txCount        		= 0xFFU;

/*******************************************************************************
 * Code
 ******************************************************************************/
void lpi2c_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_LPI2C_Nak)
    {
        nakFlag = true;
    }
}

/**
 * @brief	Read using the I2C communication protocol.
 *
 * @param	RX and TX buffers and lengths
 */
status_t rbwsI2C_Read(uint8_t* rxBuff,uint8_t* txBuff, uint8_t rxlength, uint8_t txlength)
{
		status_t reVal = kStatus_Fail;
		status_t write = kStatus_Fail;
		write = rbwsI2C_Write(txBuff, txlength);

	    if (kStatus_Success == write && kStatus_Success == LPI2C_MasterStart(LPI2C_MASTER, LPI2C_SLAVE_ADDR_7BIT, kLPI2C_Read))
	    {
	    	        reVal = LPI2C_MasterReceive(LPI2C_MASTER, rxBuff, rxlength);
	    	        if (reVal != kStatus_Success)
	    	        {
	    	            if (reVal == kStatus_LPI2C_Nak)
	    	            {
	    	                LPI2C_MasterStop(LPI2C_MASTER);
	    	            }
	    	            return -1;
	    	        }

	    	        reVal = LPI2C_MasterStop(LPI2C_MASTER);
	    	        if (reVal != kStatus_Success)
	    	        {
	    	            return -1;
	    	        }
	    	        return kStatus_Success;
	    }
	    return kStatus_Fail;
}

/**
 * @brief	Write using the I2C communication protocol.
 *
 * @param	TX buffer and length.
 */
status_t rbwsI2C_Write(uint8_t* txbuff, uint8_t length){
	status_t reVal = kStatus_Fail;

    if (kStatus_Success == LPI2C_MasterStart(LPI2C_MASTER, LPI2C_SLAVE_ADDR_7BIT, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(LPI2C_MASTER, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts(LPI2C_MASTER, NULL, &txCount);
        }

        reVal = LPI2C_MasterSend(LPI2C_MASTER,txbuff,length);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(LPI2C_MASTER);
            }
            return -1;
        }
        reVal = LPI2C_MasterStop(LPI2C_MASTER);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
        return kStatus_Success;
    }
    return kStatus_Fail;
}
