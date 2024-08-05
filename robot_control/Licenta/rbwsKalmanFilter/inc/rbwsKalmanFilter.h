/*
 * rbwsKalmanFilter.h
 *
 *  Created on: Feb 26, 2024
 *      Author: PLE1CLJ
 */

#ifndef RBWS_KALMAN_FILTER_H
#define RBWS_KALMAN_FILTER_H

#include "rbwsMPU6050.h"
/*********************/
/* Macro definitions */
/*********************/
#define		g		((float) 9.81f)

/********************/
/* Type definitions */
/********************/
typedef struct EKF
{
	float x;
	float y;
	float yaw;

	float P[2][2]; 	/* predicted covariance estimate matrix */
	float Q[3][3]; 	/* state model noise covariance matrix */
	float R[3][3]; 	/* sensor measurement noise covariance matrix */
} EKF;

/***********************/
/* Function prototypes */
/***********************/
void rbwsKalmanFilter_Init(EKF *ekf);
void rbwsKalmanFilter_Predict(EKF *ekf, IMUMeasurement measurement);
void rbwsKalmanFilter_Update(EKF *ekf, EKF* ekfPrev, IMUMeasurement measurement);

#endif /* RBWS_KALMAN_FILTER_H */
