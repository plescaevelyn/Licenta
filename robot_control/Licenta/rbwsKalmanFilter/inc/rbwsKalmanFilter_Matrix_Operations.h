/*
 * rbwsKalmanFilter_Matrix_Operations.h
 *
 *  Created on: Mar 21, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSKALMANFILTER_MATRIX_OPERATIONS_H_
#define INC_RBWSKALMANFILTER_MATRIX_OPERATIONS_H_

void sum(uint8_t *n, float (*A)[n[1]], float (*B)[n[1]], float (*C)[n[1]]);
void difference(uint8_t *n, float (*A)[n[1]], float (*B)[n[1]], float (*C)[n[1]]);
void multiply_by_scalar(uint8_t* n, float (*A)[n[1]], float scalar);
void multiply(uint8_t* n, uint8_t* m, float (*A)[m[1]], float (*B)[n[1]], float (*C)[n[1]]);
void print_matrix(float *matrix, int *n);
void transpose(float *initial_matrix, float *transposed_matrix, int *n);
void inverse(uint8_t n, float (*A)[n], float (*inv)[n]);

#endif /* INC_RBWSKALMANFILTER_MATRIX_OPERATIONS_H_ */
