/*
 * rbwsKalmanFilter_Matrix_Operations.c
 *
 *  Created on: Mar 21, 2024
 *      Author: PLE1CLJ
 */

/************/
/* Includes */
/************/
#include "stdio.h"
#include "stdint.h"
#include "math.h"

#include <fsl_common.h>
#include "fsl_debug_console.h"

/****************************/
/* Function implementations */
/****************************/
/**
 * @brief Sum two matrices A and B in order to get the result C, all of them of size n[0] x n[1]
 */
void sum(uint8_t *n, float (*A)[n[1]], float (*B)[n[1]], float (*C)[n[1]])
{
    for (int i = 0; i < n[0]; i++)
    {
        for (int j = 0; j < n[1]; j++)
        {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

/**
 * @brief The difference between two matrices A and B in order to get the result C, all of them of size n[0] x n[1]
 */
void difference(uint8_t *n, float (*A)[n[1]], float (*B)[n[1]], float (*C)[n[1]])
{
    for (int i = 0; i < n[0]; i++)
    {
        for (int j = 0; j < n[1]; j++)
        {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

/**
 * @brief Multiply matrix A of size n[0] x n[1] by a scalar
 */
void multiply_by_scalar(uint8_t* n, float (*A)[n[1]], float scalar)
{
    for (int i = 0; i < n[0]; i++) {
        for (int j = 0; j < n[1]; j++) {
            A[i][j] *= scalar;
        }
    }
}

/**
 * @brief Multiply matrices A and B of size n[0] x n[1] to get matrix C
 */
void multiply(uint8_t n0, uint8_t n1, uint8_t m1, float *A, float *B, float *C) {
    for (int i = 0; i < n0; i++) {
        for (int j = 0; j < m1; j++) {
            float sum = 0;
            for (int k = 0; k < n1; k++) {
                sum += A[i * n1 + k] * B[k * m1 + j];
            }
            C[i * m1 + j] = sum;
        }
    }
}

/**
 * @brief This function prints a matrix
 */
void print_matrix(float *matrix, int *n)
{
    for (int i = 0; i < n[0]; i++)
    {
        for (int j = 0; j < n[1]; j++)
        {
            PRINTF("%f ", matrix[i * n[1] + j]);
        }
        printf("\n");
    }
}

/**
 * @brief This function returns the transpose of a matrix of size n[0] x n[1]
 */
void transpose(float *initial_matrix, float *transposed_matrix, int *n)
{
    for (int i = 0; i < n[0]; ++i) {
        for (int j = 0; j < n[1]; ++j) {
        	transposed_matrix[j * n[0] + i] = initial_matrix[i * n[1] + j];
        }
    }
}

/**
 * @brief This function returns the inverse of a matrix A of size n x n
 */
void inverse(uint8_t n, float (*A)[n], float (*inv)[n])
{
    for (uint8_t i = 0; i < n; i++)
    {
        for (uint8_t j = 0; j < n; j++)
        {
            inv[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (uint8_t i = 0; i < n; i++)
    {
        float pivot = A[i][i];

        for (uint8_t j = 0; j < n; j++)
        {
            A[i][j]   /= pivot;
            inv[i][j] /= pivot;
        }

        for (uint8_t k = 0; k < n; k++)
        {
            if (k != i) {
                float factor = A[k][i];

                for (uint8_t j = 0; j < n; j++)
                {
                    A[k][j]	  -= factor * A[i][j];
                    inv[k][j] -= factor * inv[i][j];
                }
            }
        }
    }
}
