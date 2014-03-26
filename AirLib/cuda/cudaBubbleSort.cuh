#ifndef CUDA_BUBBLESORT_H
#define CUDA_BUBBLESORT_H

#include "cudaDefinitions.cuh"

typedef long long int64; 
typedef unsigned long long uint64;

cudaError_t sortWithCuda(float *a, int *ind, size_t size, float* time);

int doSorting();

#endif