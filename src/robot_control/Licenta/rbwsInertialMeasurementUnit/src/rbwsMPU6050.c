/*
* @file rbwsIMU_MPU6050.c
*
* @brief This module contains the functions needed for the MPU6050 Inertial Measurement Unit.
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

/************/
/* Includes */
/************/
#include "rbwsI2C.h"
#include "rbwsMPU6050.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "fsl_lpi2c.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"

/************************/
/* Variable definitions */
/************************/
uint8_t g_master_txBuff[1];
uint8_t g_master_rxBuff[1];

status_t statusRead = kStatus_Fail;
status_t statusWrite = kStatus_Fail;
status_t getMPU6050Data = kStatus_Fail;
uint8_t  connectionStatus;

uint16_t gyro_sensitivity = 131;
uint16_t acc_sensitivity = 16384;

int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
float 	accel_xf, accel_yf, accel_zf, gyro_xf, gyro_yf, gyro_zf;

float   past_meas_accel_xf, past_meas_accel_yf, past_meas_accel_zf;
float	past_meas_gyro_xf, past_meas_gyro_yf, past_meas_gyro_zf;

float	phiHat_rad = 0, thetaHat_rad = 0;


IMUMeasurement *measurement;

/* 'A^-1' matrix from Magneto */
/* float A[3][3] = {
		{1.590565, 0.090734, 0.087648},
		{0.090734, 1.308956, 0.130654},
		{0.087648, 0.130654, 1.577745}
}; */

float A[3][3] = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
};
/* 'Combined bias (b)' vector from Magneto */
float b[3] = {0.345086, 0.245105, 0.418131};

/****************************/
/* Function implementations */
/****************************/
/**
 * @brief	This function implements a delay in x ms.
 *
 * @param	ms the number of milliseconds of the delay.
 */
void delay(int16_t ms)
{
	SDK_DelayAtLeastUs(ms * 1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}

/**
 * @brief	This function gets the device ID.
 *
 * @return	g_master_rxBuff[0] is the ID.
 */
uint16_t rbwsMPU6050_GetDeviceID()
{
	g_master_txBuff[0] = MPU6050_RA_WHO_AM_I;
	rbwsI2C_Read(g_master_rxBuff, g_master_txBuff, 1, 1);

	return(g_master_rxBuff[0]);
}

/**
 * @brief	This function tests the connection of the device.
 *
 * @return	The status of the test connection (success or fail).
 */
uint8_t rbwsMPU6050_TestConnection()
{
	return kStatus_Success;
}

/**
 * @brief	This function configures MPU6050.
 *
 * @param	Register adress, mask, start position, bit value
 */
void rbwsMPU6050_Config(uint8_t register_adress, uint8_t mask, uint8_t start_position, uint8_t bit_value)
{
	uint8_t txReg[2];
	g_master_txBuff[0] = register_adress;
	statusRead = rbwsI2C_Read(g_master_rxBuff, g_master_txBuff, 1, 1);

    txReg[0] = register_adress;
    txReg[1] = g_master_rxBuff[0];

	if (statusRead == kStatus_Success) {
	    txReg[1] &= ~(mask << start_position);
	    txReg[1] |= (bit_value << start_position);
	    statusWrite = rbwsI2C_Write(txReg, 2);
	}
}

/**
 * @brief   This function configures the clock.
 */
void rbwsMPU6050_ClockConfig(void)
{
	rbwsMPU6050_Config(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_MGMT_1_SLEEP_BIT_MASK, 0x06, 0x00);
	rbwsMPU6050_Config(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLKSEL_MASK, 0x00, 0x01);
}


/**
 * @brief	This function configures the gyroscope.
 */
void rbwsMPU6050_GyroConfig(void)
{
	rbwsMPU6050_Config(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_SEL_MASK, 0x03, 0x00);
}

/**
 * @brief	This function configures the accelerometer.
 */
void rbwsMPU6050_AccelConfig(void)
{
	rbwsMPU6050_Config(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_AFS_MASK, 0x03, 0x00);
}

/**
 *	@brief	This function returns the status of whether the data is ready or not.
 *
 *	@return	status_imu returns the status of the data
 */
status_t rbwsMPU6050_DataIsReady(void)
{
	g_master_txBuff[0] = MPU6050_RA_INT_STATUS;

	uint8_t status_bit = g_master_rxBuff[1];
	status_bit &= ~(MPU6050_INT_ENABLE_DATA_RDY_INT << 0x00);

	if(status_bit == 1)
		return kStatus_Success;

	return kStatus_Fail;
}

/**
 * @brief	This function initializes MPU6050.
 */
void rbwsMPU6050_init(void)
{
	uint8_t txBuff[2] = {};
	connectionStatus = rbwsMPU6050_TestConnection();

	if(connectionStatus == kStatus_Success)
	{
		rbwsMPU6050_ClockConfig();
		rbwsMPU6050_GyroConfig();
		rbwsMPU6050_AccelConfig();

		txBuff[0] = MPU6050_RA_FIFO_EN;
		txBuff[1] = 0x00;
		rbwsI2C_Write(txBuff, 2);

		// Disable all interrupts
		txBuff[0] = MPU6050_RA_INT_ENABLE;
		txBuff[1] = 0x00;
		rbwsI2C_Write(txBuff, 2);
	}

//	rbwsKalmanFilter_Init(ekf);
//	rbwsKalmanFilter_Init(ekfPrev);
}

/**
 * @brief	This function reads the measurements of the accelerometer and gyroscope.
 *
 * @param	x, y, z data of the accelerometer and gyroscope.
 */
void rbwsMPU6050_ReadMeasurements(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z, int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z)
{
	uint8_t measurement_buffer[14];

	if(rbwsMPU6050_TestConnection() == kStatus_Success)
	{
		g_master_txBuff[0] = MPU6050_RA_ACCEL_XOUT_H;
		statusRead = rbwsI2C_Read(measurement_buffer, g_master_txBuff, 14, 1);

		*accel_x = (((int16_t)measurement_buffer[0]) << 8)  | measurement_buffer[1];
	    *accel_y = (((int16_t)measurement_buffer[2]) << 8)  | measurement_buffer[3];
	    *accel_z = (((int16_t)measurement_buffer[4]) << 8)  | measurement_buffer[5];
	    *gyro_x  = (((int16_t)measurement_buffer[8]) << 8)  | measurement_buffer[9];
	    *gyro_y  = (((int16_t)measurement_buffer[10]) << 8) | measurement_buffer[11];
	    *gyro_z  = (((int16_t)measurement_buffer[12]) << 8) | measurement_buffer[13];
	}
}

/**
 *
 */
void rbwsMPU6050_Complementary_Filter()
{
	/*
	 * COMPLEMENTARY FILTER
	 */
	/* Estimate angles using filtered accelerometer measurements */
	float phiHat_accel_rad = atanf(accel_yf / accel_zf); /* computing the accelerometer angle in radians */
	float thetaHat_accel_rad = asinf(accel_xf / GRAVITATIONAL_CONSTANT);

	/* Transform body rates into Euler rates */
	float phiDot_rps = gyro_xf + tanf(thetaHat_accel_rad) * (phiHat_rad) * gyro_yf + cosf(phiHat_rad) * gyro_zf;
	float thetaDot_rps = cosf(phiHat_rad) * gyro_yf - sinf(phiHat_rad) * gyro_zf;

	/* Combine accelerometer estimates with integral of gyro readings */
	phiHat_rad = COMP_FILT_ALPHA * phiHat_accel_rad + (1 - COMP_FILT_ALPHA) * (phiHat_accel_rad + (SAMPLE_TIME_MS / 1000.0f) * phiDot_rps);
	thetaHat_rad = COMP_FILT_ALPHA * thetaHat_accel_rad + (1 - COMP_FILT_ALPHA) * (thetaHat_accel_rad + (SAMPLE_TIME_MS / 1000.0f) * thetaDot_rps);

	PRINTF("Roll and pitch angles: %f, %f\n\n", phiHat_rad * RAD2DEG, thetaHat_rad * RAD2DEG);
}

/*
 * @brief	This function measures the surrounding data and prints them.
 */
void rbwsMPU6050_main()
{
	/*
	 * MEASURE & CALIBRATE
	 */
	rbwsMPU6050_ReadMeasurements(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

	/* Handling accelerometer data */
	/* Subtract bias */
	accel_xf = (float)accel_x / acc_sensitivity - b[0];
	accel_yf = (float)accel_y / acc_sensitivity - b[1];
	accel_zf = (float)accel_z / acc_sensitivity - b[2];

	/* Filtering the accelerometer measurements using a Low Pass Filter */
	accel_xf = past_meas_accel_xf * COMP_FILT_ALPHA + accel_xf * (1 - COMP_FILT_ALPHA);
	accel_yf = past_meas_accel_yf * COMP_FILT_ALPHA + accel_yf * (1 - COMP_FILT_ALPHA);
	accel_zf = past_meas_accel_zf * COMP_FILT_ALPHA + accel_zf * (1 - COMP_FILT_ALPHA);

	/* Handling gyroscope data */
	gyro_xf = (float)gyro_x / gyro_sensitivity; /* p */
	gyro_yf = (float)gyro_y / gyro_sensitivity; /* q */
	gyro_zf = (float)gyro_z / gyro_sensitivity; /* r */

	/* Filtering the gyroscope measurements using a Low Pass Filter */
	gyro_xf = past_meas_gyro_xf * COMP_FILT_ALPHA + gyro_xf * (1 - COMP_FILT_ALPHA);
	gyro_yf = past_meas_gyro_yf * COMP_FILT_ALPHA + gyro_yf * (1 - COMP_FILT_ALPHA);
	gyro_zf = past_meas_gyro_zf * COMP_FILT_ALPHA + gyro_zf * (1 - COMP_FILT_ALPHA);

	/* Assigning new past measurements */
	past_meas_accel_xf = accel_xf;
	past_meas_accel_yf = accel_yf;
	past_meas_accel_zf = accel_zf;

	past_meas_gyro_xf = gyro_xf;
	past_meas_gyro_yf = gyro_yf;
	past_meas_gyro_zf = gyro_zf;

	PRINTF("IMU: %f %f %f %f %f %f \n", accel_xf, accel_yf, accel_zf, gyro_xf, gyro_yf, gyro_zf);
}

/**
 * This function saves the measurements inside a struct.
 */
void rbwsInertialMeasurementUnit_Save_Measurement(IMUMeasurement *measurement,
		float accel_xf, float accel_yf, float accel_zf,
		float gyro_xf, float gyro_yf, float gyro_zf)
{
	measurement->accel_xf = accel_xf;
	measurement->accel_yf = accel_yf;
	measurement->accel_zf = accel_zf;
	measurement->gyro_xf = gyro_xf;
	measurement->gyro_yf = gyro_yf;
	measurement->gyro_zf = gyro_zf;
}

IMUMeasurement* rbwsInertialMeasurementUnit_Get_Measurements()
{
	return measurement;
}
