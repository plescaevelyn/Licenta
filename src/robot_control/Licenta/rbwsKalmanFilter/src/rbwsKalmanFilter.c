/*
* @file rbwsKalmanFilter.c
*
* @brief This module contains the Extended Kalman Filter implementation.
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
#include "rbwsKalmanFilter.h"
#include "rbwsKalmanFilter_Matrix_Operations.h"
#include "rbwsMPU6050.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "fsl_common.h"
#include "fsl_debug_console.h"

/************************/
/* Variable definitions */
/************************/
float state_estimate_t_minus_1[3] = {0, 0, 0}; /* the estimated state vector at time t-1 in the global reference frame */
float control_vector_t_minus_1[2] = {0, 0}; /* the control input vector at time t-1 in the global reference frame */

float state_estimate_t[3]; /* the estimated state vector at time t in the global reference frame */
float control_vector_t[2]; /* the control input vector at time t in the global reference frame */

float estimated_sensor_observation_y_t[3];

float A_t_minus_1[3][3]; /* expresses how the state of the system changes from t-1 to t when no control command is executed */
float B[3][2]; /* expresses how the state of the system changes from t-1 to t due to control commands */

float H[2][3]; /* Jacobian matrix */
float H_t[3][2]; /* transposed Jacobian matrix */

float S_t[2][2]; /* residual covariance matrix at time t */
float S_t_minus_1[2][2]; /* residual covariance matrix at time t-1 */
float S_inv[2][2]; /* inverse of the residual covariance matrix */

float K[2][3]; /* near-optimal Kalman gain */

float estimated_sensor_observation[3];

/****************************/
/* Function implementations */
/****************************/
/**
 * @brief Function used for defining the state space model.
 */
void rbwsKalmanFilter_getBMatrix(EKF *ekf, float dt)
{
	B[0][0] = cos(ekf->yaw) * dt;
	B[0][1] = 0;
	B[1][0] = sin(ekf->yaw) * dt;
	B[1][1] = 0;
	B[2][0] = 0;
	B[2][1] = dt;
}

/*
 * @brief This function calculated the state estimate and updates the global variable.
 * @info A state space model or state transition model is a mathematical equation that helps you estimate
 *		 the state of a system at time t given the state of a system at time t-1.
 */
void rbwsKalmanFilter_get_State_Estimate(EKF *ekf)
{
	float first_element[3][1];
	float second_element[3][1];

	uint8_t size1[2];
	uint8_t size2[2];

	size1[0] = 3; size1[1] = 3;
	size2[0] = 3; size2[1] = 1;

	rbwsKalmanFilter_getBMatrix(ekf, 1);

	multiply(size1, size2, A_t_minus_1, state_estimate_t_minus_1, first_element);
	multiply(size1, size2, B, control_vector_t_minus_1, second_element);

	sum(size1[0], first_element, second_element, state_estimate_t);
}

/**********************************/
/* Deriving the Observation Model */
/**********************************/
void rbwsKalmanFilter_get_Estimated_Sensor_Observation()
{
	uint8_t size1[2];
	uint8_t size2[2];

	size1[0] = 3; size1[1] = 3;
	size2[0] = 3; size2[1] = 1;

	multiply(size1, size2, H_t, state_estimate_t_minus_1, estimated_sensor_observation_y_t);
}

/**************************/
/* Extended Kalman Filter */
/**************************/
/**
 * @brief
 */
void rbwsKalmanFilter_Init(EKF *ekf)
{
	ekf->x = 0.0f;
	ekf->y= 0.0f;
	ekf->yaw = 0.0f;

	ekf->P[0][0] = 1.0f;
	ekf->P[0][1] = 0.0f;
	ekf->P[1][0] = 0.0f;
	ekf->P[1][1] = 1.0f;

	for (uint8_t i = 0; i < 2; i++)
	{
		for (uint8_t j = 0; j < 2; j++)
		{
			ekf->Q[i][j] 	   = (i == j) ? 1.0 : 0.0;
			ekf->P[i][j] 	   = (i == j) ? 1.0 : 0.0;
			A_t_minus_1[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

/**
 * @brief 	Extended Kalman Filter implementation of prediction.
 *
 * @param 	ekf is the instance of the EKF struct
 * @param 	measurement provides the acquired Inertial Measurement Unit measurement
 * @param 	T is the sampling time
 */
void rbwsKalmanFilter_Predict(EKF* ekf, IMUMeasurement measurement)
{
	uint8_t size1[2];
	uint8_t size2[2];

	float *A_t_minus_1_T[3][3];

	float delta_t = 1;
	size1[0] = 3; size1[1] = 3;
	size2[0] = 3; size2[1] = 3;

	transpose(A_t_minus_1, A_t_minus_1_T, size1);

	/* Predict the state estimate at time t based on the state estimate at time k-1 and the control input applied at time k-1 */
	rbwsKalmanFilter_get_State_Estimate(ekf); /* get state_estimate_k */

	rbwsKalmanFilter_getBMatrix(ekf, delta_t);

	ekf->x  += state_estimate_t[0];
	ekf->y += state_estimate_t[1];
	ekf->yaw   += state_estimate_t[2];

	/* Recompute the B matrix using the sines and cosines of the updated yaw measurements */
	rbwsKalmanFilter_getBMatrix(ekf, delta_t);

	/* Predict the state covariance estimate based on the previous covariance and noise */
	float *A_t_minus_1_P[3][3];

	multiply(size1[0], size2[0], A_t_minus_1, ekf->P, A_t_minus_1_P);
	multiply(size1[1], size2[1], A_t_minus_1_P, A_t_minus_1_T, ekf->P);
	sum(size1, ekf->P, ekf->Q, ekf->P);
}

void rbwsKalmanFilter_Update(EKF* ekf, EKF* ekfPrev, IMUMeasurement measurement)
{
	uint8_t size1[2];
	uint8_t size2[2];

	size1[0] = 2; size1[1] = 3;
	size2[0] = 3; size2[1] = 3;

	/* Calculate the Jacobian */
	H_t[0][0] = (float)(1.0f/sqrt((ekf->x - ekfPrev->x)*(ekf->x - ekfPrev->x) + (ekf->y - ekfPrev->y)*(ekf->x - ekfPrev->x)));
	H_t[0][1] = (float)(1.0f/sqrt((ekf->x - ekfPrev->x)*(ekf->x - ekfPrev->x) + (ekf->y - ekfPrev->y)*(ekf->y - ekfPrev->y)));
	H_t[0][2] = 0;
	H_t[0][0] = 0;
	H_t[1][1] = 0;
	H_t[2][2] = 0;

	/* Calculate the measurement residual covariance */
	multiply(size1, size2, H, ekf->P, S_t_minus_1);
	transpose(H, H_t, size1);
	multiply(size1, size2, S_t_minus_1, H_t, S_t);

	size1[0] = 2; size1[1] = 2;

	sum(size1, S_t, ekf->R, S_t);

	/* Calculate the near-optimal Kalman gain */
	inverse(2, S_t, S_inv);

	size1[0] = 3; size1[1] = 3;
	size2[0] = 3; size2[1] = 2;
	float P_H_t[3][2];

	multiply(size1, size2, ekf->P, H_t, P_H_t);

	size1[0] = 3; size1[1] = 2;
	size2[0] = 2; size2[1] = 2;

	multiply(size1, size2, P_H_t, S_inv, K);

	/* Calculate an updated state estimate for time k */
	float measurement_residual_y_t[3][1];
	float z_t_observation_vector[2][1] = {{measurement.v}, {measurement.gyro_zf}};
	float Hstate[2];
	float K_measurement_residual_y_t[2];

	size1[0] = 2; size1[1] = 3;
	size2[0] = 3; size2[1] = 1;

	rbwsKalmanFilter_get_State_Estimate(ekf);
	multiply(size1, size2, H, state_estimate_t, Hstate);
	size1[0]  = 2; size1[1] = 1;
	difference(size1, z_t_observation_vector, Hstate, measurement_residual_y_t);

	multiply(size1, size2, K, measurement_residual_y_t, K_measurement_residual_y_t);

	size1[1] = 1;

	sum(size1, state_estimate_t, K_measurement_residual_y_t, state_estimate_t);

	/* Update the state covariance estimate for time k */
	size1[0] = 3; size1[1] = 2;
	size2[0] = 2; size2[1] = 3;
	float K_H_t[3][3];
	float K_H_t_P[3][2];

	multiply(size1, size2, K, H_t, K_H_t);

	size1[0] = 3; size1[1] = 2;
	size2[0] = 2; size2[1] = 2;
	multiply(size1, size2, K_H_t, ekf->P, K_H_t_P);
	difference(size1, ekf->P, K_H_t_P, ekf->P);
}
