////////////////////////////////////////////////////////////////////////////
//
// Copyright 1993-2013 NVIDIA Corporation.  All rights reserved.
//
// Please refer to the NVIDIA end user license agreement (EULA) associated
// with this source code for terms and conditions that govern your use of
// this software. Any use, reproduction, disclosure, or distribution of
// this software and related documentation outside the terms of the EULA
// is strictly prohibited.
//
////////////////////////////////////////////////////////////////////////////

//
// Matrix multiplication: C = A * B.
// Host code.
//
// This sample implements matrix multiplication as described in Chapter 3
// of the programming guide and uses the CUBLAS library to demonstrate
// the best performance.

// SOME PRECAUTIONS:
// IF WE WANT TO CALCULATE ROW-MAJOR MATRIX MULTIPLY C = A * B,
// WE JUST NEED CALL CUBLAS API IN A REVERSE ORDER: cublasSegemm(B, A)! 
// The reason is explained as follows:

// CUBLAS library uses column-major storage, but C/C++ use row-major storage.
// When passing the matrix pointer to CUBLAS, the memory layout alters from 
// row-major to column-major, which is equivalent to an implict transpose. 

// In the case of row-major C/C++ matrix A, B, and a simple matrix multiplication 
// C = A * B, we can't use the input order like cublasSgemm(A, B)  because of
// implict transpose. The actual result of cublasSegemm(A, B) is A(T) * B(T). 
// If col(A(T)) != row(B(T)), equal to row(A) != col(B), A(T) and B(T) are not 
// multipliable. Moreover, even if A(T) and B(T) are multipliable, the result C 
// is a column-based cublas matrix, which means C(T) in C/C++, we need extra 
// transpose code to convert it to a row-based C/C++ matrix.

// To solve the problem, let's consider our desired result C, a row-major matrix. 
// In cublas format, it is C(T) actually (becuase of the implict transpose). 
// C = A * B, so C(T) = (A * B) (T) = B(T) * A(T). Cublas matrice B(T) and A(T) 
// happen to be C/C++ matrice B and A (still becuase of the implict transpose)! 
// We don't need extra transpose code, we only need alter the input order!
//
// CUBLAS provides high-performance matrix multiplication.
// See also:
// V. Volkov and J. Demmel, "Benchmarking GPUs to tune dense linear algebra,"
// in Proc. 2008 ACM/IEEE Conf. on Superconducting (SC '08),
// Piscataway, NJ: IEEE Press, 2008, pp. Art. 31:1-11.
//

#ifndef CUDA_MATMUL_H
#define CUDA_MATMUL_H

#include "cudaDefinitions.cuh"

// Utilities and system includes
#include <assert.h>
#include <helper_string.h>  // helper for shared functions common to CUDA SDK samples

// CUDA runtime
#include <cuda_runtime.h>
#include <cublas_v2.h>

// CUDA and CUBLAS functions
#include <helper_functions.h>

#ifndef min
#define min(a,b) ((a < b) ? a : b)
#endif
#ifndef max
#define max(a,b) ((a > b) ? a : b)
#endif

typedef struct _matrixSize      // Optional Command-line multiplier for matrix sizes
{
    unsigned int uiWA, uiHA, uiWB, uiHB, uiWC, uiHC;
} sMatrixSize;

////////////////////////////////////////////////////////////////////////////////
//! Compute reference data set matrix multiply on CPU
//! C = A * B
//! @param C          reference data, computed but preallocated
//! @param A          matrix A as provided to device
//! @param B          matrix B as provided to device
//! @param hA         height of matrix A
//! @param wB         width of matrix B
////////////////////////////////////////////////////////////////////////////////
void
matrixMulCPU(float *C, const float *A, const float *B, unsigned int hA, unsigned int wA, unsigned int wB);

////////////////////////////////////////////////////////////////////////////////
// These are CUDA Helper functions (in addition to helper_cuda.h)

void inline checkError(cublasStatus_t status, const char *msg);
// end of CUDA Helper Functions

// Allocates a matrix with random float entries.
void randomInit(float *data, int size);

void printDiff(float *data1, float *data2, int width, int height, int iListLength, float fListTol);

void initializeCUDA(int &devID, int &iSizeMultiple, sMatrixSize &matrix_size);

////////////////////////////////////////////////////////////////////////////////
//! Run a simple test matrix multiply using CUBLAS
////////////////////////////////////////////////////////////////////////////////
int matrixMultiply(int devID, sMatrixSize &matrix_size);

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int lanzarProceso();


#endif