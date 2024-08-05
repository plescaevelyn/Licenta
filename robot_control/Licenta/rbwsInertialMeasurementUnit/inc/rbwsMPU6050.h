/*
 * rbwsIMU_MPU6050.h
 *
 *  Created on: Feb 13, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSMPU6050_H_
#define INC_RBWSMPU6050_H_

#include "fsl_common.h"

/*********************/
/* Macro definitions */
/*********************/
#define SAMPLE_TIME_MS								20
#define COMP_FILT_ALPHA								0.05f
#define PI											3.14159265
#define RAD2DEG										180/PI
#define DEG2RAD										PI/180
#define GRAVITATIONAL_CONSTANT						9.81f

#define MCU6050_ADDRESS_AD0_LOW						0x68
#define MCU6050_ADDRESS_AD0_HIGH					0x69
#define MCU6050_BASE_ADRESS							MCU6050_ADDRESS_AD0_LOW

#define MPU6050_SELF_TEST_X							0x0D
#define MPU6050_SELF_TEST_Y							0x0E
#define MPU6050_SELF_TEST_Z							0x0F
#define MPU6050_SELF_TEST_A							0x10
#define MPU6050_RA_XG_OFFSET_H      				0x13
#define MPU6050_RA_XG_OFFSET_L      				0x14
#define MPU6050_RA_YG_OFFSET_H      				0x15
#define MPU6050_RA_YG_OFFSET_L      				0x16
#define MPU6050_RA_ZG_OFFSET_H      				0x17
#define MPU6050_RA_ZG_OFFSET_L      				0x18
#define MPU6050_RA_SMPLRT_DIV       				0x19
#define MPU6050_RA_CONFIG           				0x1A
#define MPU6050_RA_GYRO_CONFIG      				0x1B
#define MPU6050_RA_ACCEL_CONFIG     				0x1C
#define MPU6050_RA_FIFO_EN          				0x23
#define MPU6050_RA_I2C_MST_CTRL						0x24
#define MPU6050_RA_I2C_SLV0_ADDR    				0x25
#define MPU6050_RA_I2C_SLV0_REG     				0x26
#define MPU6050_RA_I2C_SLV0_CTRL    				0x27
#define MPU6050_RA_INT_PIN_CFG      				0x37
#define MPU6050_RA_INT_ENABLE       				0x38
#define MPU6050_RA_DMP_INT_STATUS   				0x39
#define MPU6050_RA_INT_STATUS       				0x3A
#define MPU6050_RA_ACCEL_XOUT_H     				0x3B
#define MPU6050_RA_ACCEL_XOUT_L     				0x3C
#define MPU6050_RA_ACCEL_YOUT_H     				0x3D
#define MPU6050_RA_ACCEL_YOUT_L     				0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     				0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     				0x40
#define MPU6050_RA_TEMP_OUT_H       				0x41
#define MPU6050_RA_TEMP_OUT_L       				0x42
#define MPU6050_RA_GYRO_XOUT_H      				0x43
#define MPU6050_RA_GYRO_XOUT_L    					0x44
#define MPU6050_RA_GYRO_YOUT_H      				0x45
#define MPU6050_RA_GYRO_YOUT_L      				0x46
#define MPU6050_RA_GYRO_ZOUT_H      				0x47
#define MPU6050_RA_GYRO_ZOUT_L      				0x48
#define MPU6050_RA_USER_CTRL        				0x6A
#define MPU6050_RA_PWR_MGMT_1       				0x6B
#define MPU6050_RA_PWR_MGMT_2       				0x6C
#define MPU6050_RA_FIFO_COUNT						0x72
#define MPU6050_RA_FIFO_R_W         				0x74
#define MPU6050_RA_WHO_AM_I         				0x75
#define MPU6050_RA_XA_OFFSET_H      				0x77
#define MPU6050_RA_XA_OFFSET_L      				0x78
#define MPU6050_RA_YA_OFFSET_H      				0x7A
#define MPU6050_RA_YA_OFFSET_L      				0x7B
#define MPU6050_RA_ZA_OFFSET_H      				0x7D
#define MPU6050_RA_ZA_OFFSET_L      				0x7E

#define MPU6050_PWR_MGMT_1_CLKSEL_MASK 				0x07
#define MPU6050_PWR1_MGMT_1_SLEEP_BIT_MASK          0x01
#define MPU6050_PWR1_MGMT_1_RESET_DEVICE_MASK       0x01

#define MPU6050_RESET_REGISTER_MASK					0XFF
#define MPU6050_GYRO_CONFIG_FS_SEL_MASK				0x03
#define MPU6050_ACCEL_CONFIG_AFS_MASK				0x03
#define MPU6050_SMPLRT_DIV_MASK						0x7F
#define MPU6050_CONFIG_DLP_CFG						0x07
#define MPU6050_INT_ENABLE_DATA_RDY_EN				0x01
#define MPU6050_INT_ENABLE_DATA_RDY_INT				0x01
#define MPU6050_CLOCK_PLL_XGYRO						0x01

#define MPU6050_NO_DATA								kStatus_Fail
#define MPU6050_DATA_RECEIVED						kStatus_Success

/********************/
/* Type definitions */
/********************/
typedef struct IMUMeasurement
{
    float accel_xf;
    float accel_yf;
    float accel_zf;

    float gyro_xf;
    float gyro_yf;
    float gyro_zf;

    float v;
} __attribute__((packed)) IMUMeasurement;

/***********************/
/* Function prototypes */
/***********************/
void rbwsMPU6050_init(void);
void rbwsMPU6050_Config(uint8_t regAddr, uint8_t mask, uint8_t startPosition, uint8_t bitValue);
void rbwsMPU6050_ClockConfig(void);
void rbwsMPU6050_AccelConfig(void);
void rbwsMPU6050_GyroConfig(void);

status_t rbwsMPU6050_DataIsReady(void);
void rbwsMPU6050_ReadMeasurements(int16_t* accel_x, int16_t* accel_y,int16_t* accel_z, int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z);
void rbwsMPU6050_getXAccelOffset();
void rbwsMPU6050_main();
void rbwsInertialMeasurementUnit_Save_Measurement(IMUMeasurement *measurement, float accel_xf, float accel_yf, float accel_zf,float gyro_xf, float gyro_yf, float gyro_zf);

uint16_t rbwsMPU6050_GetDeviceID();
uint8_t rbwsMPU6050_TestConnection();

#endif /* INC_RBWSMPU6050_H_ */
